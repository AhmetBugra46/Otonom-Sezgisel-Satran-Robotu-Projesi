#!/usr/bin/env python3
import sys
import random
import time
import json
import os
import copy
import math
import threading

import rclpy
from rclpy.node import Node
from xarm_msgs.srv import PlanPose, PlanExec, GripperMove
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene, ObjectColor
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ==============================================================================
# FİDE STANDARTLARI VE MEKATRONİK AYARLAR
# ==============================================================================
SQUARE_SIZE = 0.050  
PIECE_RADIUS = 0.009 
X_BOARD_START = 0.26 
Y_BOARD_START = -0.20

# ==============================================================================
# 1. BÖLÜM: ORİJİNAL SATRANÇ MOTORU (EKSİKSİZ KORUNDU)
# ==============================================================================
class CastleRights:
    def __init__(self, wks, wqs, bks, bqs):
        self.wks = wks
        self.wqs = wqs
        self.bks = bks
        self.bqs = bqs

class GameState:
    def __init__(self):
        self.board = [
            ["bR", "bN", "bB", "bQ", "bK", "bB", "bN", "bR"],
            ["bP", "bP", "bP", "bP", "bP", "bP", "bP", "bP"],
            ["--", "--", "--", "--", "--", "--", "--", "--"],
            ["--", "--", "--", "--", "--", "--", "--", "--"],
            ["--", "--", "--", "--", "--", "--", "--", "--"],
            ["--", "--", "--", "--", "--", "--", "--", "--"],
            ["wP", "wP", "wP", "wP", "wP", "wP", "wP", "wP"],
            ["wR", "wN", "wB", "wQ", "wK", "wB", "wN", "wR"]
        ]
        self.white_to_move = True
        self.move_log = []
        self.white_king_location = (7, 4)
        self.black_king_location = (0, 4)
        self.checkmate = False
        self.stalemate = False
        self.current_castling_right = CastleRights(True, True, True, True)
        self.castle_rights_log = [CastleRights(True, True, True, True)]

    def clone(self):
        return copy.deepcopy(self)

    def generate_pgn(self):
        pgn = ""
        turn = 1
        for i, move in enumerate(self.move_log):
            if i % 2 == 0:
                pgn += f"{turn}. {move.get_chess_notation()} "
            else:
                pgn += f"{move.get_chess_notation()} "
                turn += 1
        return pgn.strip()

    def get_fen(self):
        fen = ""
        for r in range(8):
            empty = 0
            for c in range(8):
                if self.board[r][c] == "--":
                    empty += 1
                else:
                    if empty > 0:
                        fen += str(empty)
                        empty = 0
                    color = self.board[r][c][0]
                    piece = self.board[r][c][1]
                    fen += piece.upper() if color == 'w' else piece.lower()
            if empty > 0:
                fen += str(empty)
            if r < 7:
                fen += "/"
        fen += " w " if self.white_to_move else " b "
        fen += "KQkq - 0 1"
        return fen

    def count_pieces(self):
        counts = {"wQ":0, "wR":0, "wB":0, "wN":0, "wP":0, "bQ":0, "bR":0, "bB":0, "bN":0, "bP":0}
        for r in range(8):
            for c in range(8):
                p = self.board[r][c]
                if p != "--" and p[1] != 'K':
                    counts[p] += 1
        return counts

    def get_center_control(self):
        center_sqs = [(4,3), (4,4), (3,3), (3,4)]
        w_ctrl = 0
        b_ctrl = 0
        for r, c in center_sqs:
            p = self.board[r][c]
            if p[0] == 'w':
                w_ctrl += 1
            elif p[0] == 'b':
                b_ctrl += 1
        return w_ctrl, b_ctrl

    def make_move(self, move):
        self.board[move.start_row][move.start_col] = "--"
        self.board[move.end_row][move.end_col] = move.piece_moved
        if move.piece_moved == 'wP' and move.end_row == 0:
            self.board[move.end_row][move.end_col] = 'wQ'
            move.is_pawn_promotion = True
        elif move.piece_moved == 'bP' and move.end_row == 7:
            self.board[move.end_row][move.end_col] = 'bQ'
            move.is_pawn_promotion = True

        if move.is_castle_move:
            if move.end_col - move.start_col == 2: 
                self.board[move.end_row][move.end_col-1] = self.board[move.end_row][move.end_col+1]
                self.board[move.end_row][move.end_col+1] = "--"
            else: 
                self.board[move.end_row][move.end_col+1] = self.board[move.end_row][move.end_col-2]
                self.board[move.end_row][move.end_col-2] = "--"

        self.move_log.append(move)
        self.update_castle_rights(move)
        self.castle_rights_log.append(CastleRights(
            self.current_castling_right.wks, 
            self.current_castling_right.wqs, 
            self.current_castling_right.bks, 
            self.current_castling_right.bqs
        ))

        if move.piece_moved == 'wK':
            self.white_king_location = (move.end_row, move.end_col)
        elif move.piece_moved == 'bK':
            self.black_king_location = (move.end_row, move.end_col)

        self.white_to_move = not self.white_to_move

    def undo_move(self):
        if len(self.move_log) != 0:
            move = self.move_log.pop()
            self.board[move.start_row][move.start_col] = move.piece_moved
            self.board[move.end_row][move.end_col] = move.piece_captured
            self.white_to_move = not self.white_to_move

            if move.piece_moved == 'wK':
                self.white_king_location = (move.start_row, move.start_col)
            elif move.piece_moved == 'bK':
                self.black_king_location = (move.start_row, move.start_col)

            if move.is_castle_move:
                if move.end_col - move.start_col == 2:
                    self.board[move.end_row][move.end_col+1] = self.board[move.end_row][move.end_col-1]
                    self.board[move.end_row][move.end_col-1] = "--"
                else:
                    self.board[move.end_row][move.end_col-2] = self.board[move.end_row][move.end_col+1]
                    self.board[move.end_row][move.end_col+1] = "--"

            self.castle_rights_log.pop()
            self.current_castling_right = copy.deepcopy(self.castle_rights_log[-1])
            self.checkmate = False
            self.stalemate = False

    def update_castle_rights(self, move):
        if move.piece_moved == 'wK':
            self.current_castling_right.wks = False
            self.current_castling_right.wqs = False
        elif move.piece_moved == 'bK':
            self.current_castling_right.bks = False
            self.current_castling_right.bqs = False
        elif move.piece_moved == 'wR':
            if move.start_row == 7:
                if move.start_col == 0:
                    self.current_castling_right.wqs = False
                elif move.start_col == 7:
                    self.current_castling_right.wks = False
        elif move.piece_moved == 'bR':
            if move.start_row == 0:
                if move.start_col == 0:
                    self.current_castling_right.bqs = False
                elif move.start_col == 7:
                    self.current_castling_right.bks = False

    def get_valid_moves(self):
        temp_castle_rights = copy.deepcopy(self.current_castling_right)
        moves = self.get_all_possible_moves()
        if self.white_to_move:
            self.get_castle_moves(self.white_king_location[0], self.white_king_location[1], moves)
        else:
            self.get_castle_moves(self.black_king_location[0], self.black_king_location[1], moves)

        for i in range(len(moves) - 1, -1, -1):
            self.make_move(moves[i])
            self.white_to_move = not self.white_to_move 
            if self.in_check():
                moves.remove(moves[i])
            self.white_to_move = not self.white_to_move 
            self.undo_move()

        if len(moves) == 0:
            if self.in_check():
                self.checkmate = True
            else:
                self.stalemate = True
        else:
            self.checkmate = False
            self.stalemate = False

        self.current_castling_right = temp_castle_rights
        return moves

    def in_check(self):
        if self.white_to_move:
            return self.square_under_attack(self.white_king_location[0], self.white_king_location[1])
        else:
            return self.square_under_attack(self.black_king_location[0], self.black_king_location[1])

    def square_under_attack(self, r, c):
        self.white_to_move = not self.white_to_move
        opp_moves = self.get_all_possible_moves()
        self.white_to_move = not self.white_to_move
        for move in opp_moves:
            if move.end_row == r and move.end_col == c:
                return True
        return False

    def get_all_possible_moves(self):
        moves = []
        for r in range(8):
            for c in range(8):
                turn = self.board[r][c][0]
                if (turn == 'w' and self.white_to_move) or (turn == 'b' and not self.white_to_move):
                    piece = self.board[r][c][1]
                    if piece == 'P':
                        self.get_pawn_moves(r, c, moves)
                    elif piece == 'R':
                        self.get_rook_moves(r, c, moves)
                    elif piece == 'N':
                        self.get_knight_moves(r, c, moves)
                    elif piece == 'B':
                        self.get_bishop_moves(r, c, moves)
                    elif piece == 'Q':
                        self.get_bishop_moves(r, c, moves)
                        self.get_rook_moves(r, c, moves)
                    elif piece == 'K':
                        self.get_king_moves(r, c, moves)
        return moves

    def get_pawn_moves(self, r, c, moves):
        if self.white_to_move:
            if self.board[r-1][c] == "--":
                moves.append(Move((r, c), (r-1, c), self.board))
                if r == 6 and self.board[r-2][c] == "--":
                    moves.append(Move((r, c), (r-2, c), self.board))
            if c-1 >= 0 and self.board[r-1][c-1][0] == 'b':
                moves.append(Move((r, c), (r-1, c-1), self.board))
            if c+1 <= 7 and self.board[r-1][c+1][0] == 'b':
                moves.append(Move((r, c), (r-1, c+1), self.board))
        else:
            if r+1 < 8:
                if self.board[r+1][c] == "--":
                    moves.append(Move((r, c), (r+1, c), self.board))
                    if r == 1 and self.board[r+2][c] == "--":
                        moves.append(Move((r, c), (r+2, c), self.board))
                if c-1 >= 0 and self.board[r+1][c-1][0] == 'w':
                    moves.append(Move((r, c), (r+1, c-1), self.board))
                if c+1 <= 7 and self.board[r+1][c+1][0] == 'w':
                    moves.append(Move((r, c), (r+1, c+1), self.board))

    def get_rook_moves(self, r, c, moves):
        directions = [(-1, 0), (0, -1), (1, 0), (0, 1)]
        enemy = "b" if self.white_to_move else "w"
        for d in directions:
            for i in range(1, 8):
                er = r + d[0] * i
                ec = c + d[1] * i
                if 0 <= er < 8 and 0 <= ec < 8:
                    p = self.board[er][ec]
                    if p == "--":
                        moves.append(Move((r, c), (er, ec), self.board))
                    elif p[0] == enemy:
                        moves.append(Move((r, c), (er, ec), self.board))
                        break
                    else:
                        break
                else:
                    break

    def get_knight_moves(self, r, c, moves):
        km = [(-2, -1), (-2, 1), (-1, -2), (-1, 2), (1, -2), (1, 2), (2, -1), (2, 1)]
        enemy = "b" if self.white_to_move else "w"
        for m in km:
            er = r + m[0]
            ec = c + m[1]
            if 0 <= er < 8 and 0 <= ec < 8:
                p = self.board[er][ec]
                if p == "--" or p[0] == enemy:
                    moves.append(Move((r, c), (er, ec), self.board))

    def get_bishop_moves(self, r, c, moves):
        directions = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        enemy = "b" if self.white_to_move else "w"
        for d in directions:
            for i in range(1, 8):
                er = r + d[0] * i
                ec = c + d[1] * i
                if 0 <= er < 8 and 0 <= ec < 8:
                    p = self.board[er][ec]
                    if p == "--":
                        moves.append(Move((r, c), (er, ec), self.board))
                    elif p[0] == enemy:
                        moves.append(Move((r, c), (er, ec), self.board))
                        break
                    else:
                        break
                else:
                    break

    def get_king_moves(self, r, c, moves):
        km = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        enemy = "b" if self.white_to_move else "w"
        for m in km:
            er = r + m[0]
            ec = c + m[1]
            if 0 <= er < 8 and 0 <= ec < 8:
                p = self.board[er][ec]
                if p == "--" or p[0] == enemy:
                    moves.append(Move((r, c), (er, ec), self.board))

    def get_castle_moves(self, r, c, moves):
        if self.square_under_attack(r, c):
            return
        if (self.white_to_move and self.current_castling_right.wks) or (not self.white_to_move and self.current_castling_right.bks):
            if self.board[r][c+1] == '--' and self.board[r][c+2] == '--':
                if not self.square_under_attack(r, c+1) and not self.square_under_attack(r, c+2):
                    moves.append(Move((r, c), (r, c+2), self.board, is_castle=True))
        if (self.white_to_move and self.current_castling_right.wqs) or (not self.white_to_move and self.current_castling_right.bqs):
            if self.board[r][c-1] == '--' and self.board[r][c-2] == '--' and self.board[r][c-3] == '--':
                if not self.square_under_attack(r, c-1) and not self.square_under_attack(r, c-2):
                    moves.append(Move((r, c), (r, c-2), self.board, is_castle=True))

class Move:
    def __init__(self, start_sq, end_sq, board, is_castle=False):
        self.start_row = start_sq[0]
        self.start_col = start_sq[1]
        self.end_row = end_sq[0]
        self.end_col = end_sq[1]
        self.piece_moved = board[self.start_row][self.start_col]
        self.piece_captured = board[self.end_row][self.end_col]
        self.is_pawn_promotion = False
        self.is_castle_move = is_castle
        self.move_id = self.start_row * 1000 + self.start_col * 100 + self.end_row * 10 + self.end_col

    def __eq__(self, other):
        if isinstance(other, Move):
            return self.move_id == other.move_id
        return False

    def get_chess_notation(self):
        return self.get_rank_file(self.start_row, self.start_col) + self.get_rank_file(self.end_row, self.end_col)

    def get_rank_file(self, r, c):
        cols = {0:'a', 1:'b', 2:'c', 3:'d', 4:'e', 5:'f', 6:'g', 7:'h'}
        rows = {0:'8', 1:'7', 2:'6', 3:'5', 4:'4', 5:'3', 6:'2', 7:'1'}
        return cols[c] + rows[r]

    def get_uci(self):
        return self.get_rank_file(self.start_row, self.start_col) + self.get_rank_file(self.end_row, self.end_col)

class OpeningBook:
    def __init__(self, brain_file="beyin.json"):
        self.book = {}
        self.load_brain(brain_file)

    def load_brain(self, filename):
        if os.path.exists(filename):
            try:
                with open(filename, "r", encoding="utf-8") as f:
                    self.book = json.load(f)
            except:
                self.book = {}

    def get_book_move(self, board_fen):
        base_fen = " ".join(board_fen.split(" ")[:4])
        if base_fen in self.book:
            try:
                return max(self.book[base_fen], key=self.book[base_fen].get)
            except:
                return None
        return None

class ChessAI:
    def __init__(self):
        self.CHECKMATE = 10000
        self.STALEMATE = 0
        self.opening_book = OpeningBook()
        self.piece_score = {"K": 0, "Q": 950, "R": 500, "B": 330, "N": 320, "P": 100}
        self.nodes_visited = 0
        
        self.pawn_table = [
            [0,  0,  0,  0,  0,  0,  0,  0],
            [50, 50, 50, 50, 50, 50, 50, 50],
            [10, 10, 20, 30, 30, 20, 10, 10],
            [5,  5, 10, 25, 25, 10,  5,  5],
            [0,  0,  0, 20, 20,  0,  0,  0],
            [5, -5,-10,  0,  0,-10, -5,  5],
            [5, 10, 10,-20,-20, 10, 10,  5],
            [0,  0,  0,  0,  0,  0,  0,  0]
        ]
        
        self.knight_table = [
            [-50,-40,-30,-30,-30,-30,-40,-50],
            [-40,-20,  0,  0,  0,  0,-20,-40],
            [-30,  0, 10, 15, 15, 10,  0,-30],
            [-30,  5, 15, 20, 20, 15,  5,-30],
            [-30,  0, 15, 20, 20, 15,  0,-30],
            [-30,  5, 10, 15, 15, 10,  5,-30],
            [-40,-20,  0,  5,  5,  0,-20,-40],
            [-50,-40,-30,-30,-30,-30,-40,-50]
        ]

    def find_best_move_smart(self, gs, valid_moves, time_limit):
        self.nodes_visited = 0
        start_time = time.time()
        book_move = self.opening_book.get_book_move(gs.get_fen())
        if book_move:
            for m in valid_moves:
                if m.get_uci() == book_move:
                    return m, 100.0, 1, 1

        best_global_move = None
        best_global_score = -float('inf')
        random.shuffle(valid_moves)
        valid_moves.sort(key=lambda m: (100 if m.piece_captured != "--" else 0), reverse=True)
        current_depth = 1
        history = []

        while True:
            if time.time() - start_time > time_limit:
                break
            try:
                best_move_this_depth = None
                best_score_this_depth = -float('inf')
                alpha = -self.CHECKMATE
                beta = self.CHECKMATE
                
                for move in valid_moves:
                    gs.make_move(move)
                    score = -self.minimax(gs, current_depth - 1, -beta, -alpha, 1, start_time, time_limit)
                    gs.undo_move()
                    if time.time() - start_time > time_limit:
                        raise TimeoutError
                    
                    if score > best_score_this_depth:
                        best_score_this_depth = score
                        best_move_this_depth = move
                    if score > alpha:
                        alpha = score
                
                best_global_move = best_move_this_depth
                best_global_score = best_score_this_depth
                print(f"🔎 Derinlik {current_depth}: {best_global_move.get_chess_notation()} ({best_global_score:.2f})")
                
                history.append(best_global_move)
                if len(history) >= 3 and history[-1] == history[-2] == history[-3]:
                    if time.time() - start_time > (time_limit * 0.4):
                        break
                if best_global_score > 9000:
                    break
                
                current_depth += 1
                if current_depth > 12:
                    break 
            except TimeoutError:
                break

        if not best_global_move and valid_moves:
            best_global_move = valid_moves[0]
        return best_global_move, best_global_score, self.nodes_visited, (current_depth - 1)

    def minimax(self, gs, depth, alpha, beta, turn_multiplier, start_time, time_limit):
        self.nodes_visited += 1
        if self.nodes_visited % 500 == 0:
            if time.time() - start_time > time_limit:
                raise TimeoutError
        
        if depth == 0:
            return turn_multiplier * self.score_board(gs)
        
        valid_moves = gs.get_valid_moves()
        if not valid_moves:
            if gs.in_check():
                return -self.CHECKMATE + depth
            else:
                return self.STALEMATE
        
        max_score = -self.CHECKMATE
        for move in valid_moves:
            gs.make_move(move)
            score = -self.minimax(gs, depth - 1, -beta, -alpha, -turn_multiplier, start_time, time_limit)
            gs.undo_move()
            
            if score > max_score:
                max_score = score
            if max_score > alpha:
                alpha = max_score
            if alpha >= beta:
                break
        return max_score

    def score_board(self, gs):
        if gs.checkmate:
            return -self.CHECKMATE if gs.white_to_move else self.CHECKMATE
        if gs.stalemate:
            return self.STALEMATE
        
        score = 0
        for r in range(8):
            for c in range(8):
                piece = gs.board[r][c]
                if piece != "--":
                    val = self.piece_score[piece[1]]
                    if piece[1] == 'P':
                        val += self.pawn_table[r][c] if piece[0] == 'w' else self.pawn_table[7-r][c]
                    elif piece[1] == 'N':
                        val += self.knight_table[r][c] if piece[0] == 'w' else self.knight_table[7-r][c]
                    
                    if piece[0] == 'w':
                        score += val
                    else:
                        score -= val
        return score

# ==============================================================================
# 2. BÖLÜM: ROS 2 ENTEGRASYON VE ÇİFT AŞAMALI DURUM MAKİNELİ TAŞIMA MOTORU
# ==============================================================================
class ChessBrainNode(Node):
    def __init__(self):
        super().__init__('chess_brain_node')
        self.get_logger().info('🤖 ROS 2 Durum Makineli & Kartezyen Satranç Sistemi Aktif!')
        self.gs = GameState()
        self.ai = ChessAI()
        
        self.plan_client = self.create_client(PlanPose, '/xarm_pose_plan')
        self.exec_client = self.create_client(PlanExec, '/xarm_exec_plan')
        self.gripper_client = self.create_client(GripperMove, '/xarm_gripper_move')
        
        self.gripper_trajectory_pub = self.create_publisher(JointTrajectory, '/xarm_gripper_traj_controller/joint_trajectory', 10)
        self.collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        self.attached_pub = self.create_publisher(AttachedCollisionObject, '/attached_collision_object', 10)
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        
        self.z_safe = 0.250        
        self.x_home = 0.18
        self.y_home = 0.0
        self.z_home = 0.15
        
        self.captured_white_pieces = []
        self.captured_black_pieces = []
        self.coord_map = self.generate_chess_board_map()

    # ✅ Parmakların masayı delmemesi için inilebilecek en düşük güvenli limitler (Taşların gövdesi)
    def get_grab_height(self, piece_type):
        p = piece_type[1]
        if p == 'P': return 0.045
        elif p in ['R', 'N']: return 0.048
        elif p == 'B': return 0.052
        elif p == 'Q': return 0.056
        elif p == 'K': return 0.062 
        return 0.045

    def generate_chess_board_map(self):
        mapping = {}
        columns = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
        for r_idx in range(8):
            for c_idx in range(8):
                sq_name = f"{columns[c_idx]}{8 - r_idx}"
                mapping[sq_name] = {
                    "x": round(X_BOARD_START + (r_idx * SQUARE_SIZE), 3), 
                    "y": round(Y_BOARD_START + (c_idx * SQUARE_SIZE), 3)
                }
        return mapping

    # ✅ Mezarlıktaki taşların arası açıldı (Parmakların dış kısımları çarpmasın diye)
    def get_graveyard_coords_by_index(self, index, is_white_piece):
        if is_white_piece: 
            return X_BOARD_START + ((index % 4) * 0.065), Y_BOARD_START - 0.06 - ((index // 4) * 0.065)
        return X_BOARD_START + ((index % 4) * 0.065), Y_BOARD_START + (SQUARE_SIZE * 8) + 0.06 + ((index // 4) * 0.065)

    def send_color_update(self, obj_id, r, g, b):
        scene = PlanningScene()
        scene.is_diff = True
        oc = ObjectColor()
        oc.id = obj_id
        oc.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0)
        scene.object_colors.append(oc)
        self.scene_pub.publish(scene)

    def spawn_advanced_piece(self, name_id, x, y, piece_type):
        obj = CollisionObject()
        obj.id = name_id
        obj.header.frame_id = "link_base"
        ptype = piece_type[1]
        z_floor = 0.015
        prad = 0.008 # Gripper rahat girsin diye 1 mm inceltildi

        if ptype == 'P':
            p1 = SolidPrimitive(); p1.type = SolidPrimitive.CYLINDER; p1.dimensions = [0.022, prad]
            pose1 = Pose(); pose1.position.x = x; pose1.position.y = y; pose1.position.z = z_floor + 0.011; pose1.orientation.w = 1.0
            obj.primitives.append(p1); obj.primitive_poses.append(pose1)
            p2 = SolidPrimitive(); p2.type = SolidPrimitive.SPHERE; p2.dimensions = [prad * 1.1]
            pose2 = Pose(); pose2.position.x = x; pose2.position.y = y; pose2.position.z = z_floor + 0.024; pose2.orientation.w = 1.0
            obj.primitives.append(p2); obj.primitive_poses.append(pose2)
        elif ptype == 'R':
            p1 = SolidPrimitive(); p1.type = SolidPrimitive.CYLINDER; p1.dimensions = [0.030, prad*1.1]
            pose1 = Pose(); pose1.position.x = x; pose1.position.y = y; pose1.position.z = z_floor + 0.015; pose1.orientation.w = 1.0
            obj.primitives.append(p1); obj.primitive_poses.append(pose1)
            offsets = [(-0.005, -0.005), (-0.005, 0.005), (0.005, -0.005), (0.005, 0.005)]
            for ox, oy in offsets:
                bp = SolidPrimitive(); bp.type = SolidPrimitive.BOX; bp.dimensions = [0.004, 0.004, 0.006]
                bpose = Pose(); bpose.position.x = x + ox; bpose.position.y = y + oy; bpose.position.z = z_floor + 0.032; bpose.orientation.w = 1.0
                obj.primitives.append(bp); obj.primitive_poses.append(bpose)
        elif ptype == 'N':
            p1 = SolidPrimitive(); p1.type = SolidPrimitive.CYLINDER; p1.dimensions = [0.012, prad]
            pose1 = Pose(); pose1.position.x = x; pose1.position.y = y; pose1.position.z = z_floor + 0.006; pose1.orientation.w = 1.0
            obj.primitives.append(p1); obj.primitive_poses.append(pose1)
            p2 = SolidPrimitive(); p2.type = SolidPrimitive.BOX; p2.dimensions = [0.013, 0.020, 0.024]
            pose2 = Pose(); pose2.position.x = x + 0.004; pose2.position.y = y; pose2.position.z = z_floor + 0.020; pose2.orientation.w = 1.0
            obj.primitives.append(p2); obj.primitive_poses.append(pose2)
            for ey in [-0.004, 0.004]:
                ep = SolidPrimitive(); ep.type = SolidPrimitive.BOX; ep.dimensions = [0.003, 0.003, 0.005]
                epose = Pose(); epose.position.x = x - 0.002; epose.position.y = y + ey; epose.position.z = z_floor + 0.032; epose.orientation.w = 1.0
                obj.primitives.append(ep); obj.primitive_poses.append(epose)
        elif ptype == 'B':
            p1 = SolidPrimitive(); p1.type = SolidPrimitive.CYLINDER; p1.dimensions = [0.032, prad*0.9]
            pose1 = Pose(); pose1.position.x = x; pose1.position.y = y; pose1.position.z = z_floor + 0.016; pose1.orientation.w = 1.0
            obj.primitives.append(p1); obj.primitive_poses.append(pose1)
            p2 = SolidPrimitive(); p2.type = SolidPrimitive.SPHERE; p2.dimensions = [prad*1.0]
            pose2 = Pose(); pose2.position.x = x; pose2.position.y = y; pose2.position.z = z_floor + 0.034; pose2.orientation.w = 1.0
            obj.primitives.append(p2); obj.primitive_poses.append(pose2)
        elif ptype == 'Q':
            p1 = SolidPrimitive(); p1.type = SolidPrimitive.CYLINDER; p1.dimensions = [0.042, prad*1.1]
            pose1 = Pose(); pose1.position.x = x; pose1.position.y = y; pose1.position.z = z_floor + 0.021; pose1.orientation.w = 1.0
            obj.primitives.append(p1); obj.primitive_poses.append(pose1)
            p2 = SolidPrimitive(); p2.type = SolidPrimitive.CYLINDER; p2.dimensions = [0.005, prad*1.4]
            pose2 = Pose(); pose2.position.x = x; pose2.position.y = y; pose2.position.z = z_floor + 0.043; pose2.orientation.w = 1.0
            obj.primitives.append(p2); obj.primitive_poses.append(pose2)
        else: # King
            p1 = SolidPrimitive(); p1.type = SolidPrimitive.CYLINDER; p1.dimensions = [0.048, prad*1.2]
            pose1 = Pose(); pose1.position.x = x; pose1.position.y = y; pose1.position.z = z_floor + 0.024; pose1.orientation.w = 1.0
            obj.primitives.append(p1); obj.primitive_poses.append(pose1)
            p2 = SolidPrimitive(); p2.type = SolidPrimitive.BOX; p2.dimensions = [0.004, 0.004, 0.012]
            pose2 = Pose(); pose2.position.x = x; pose2.position.y = y; pose2.position.z = z_floor + 0.054; pose2.orientation.w = 1.0
            obj.primitives.append(p2); obj.primitive_poses.append(pose2)
            p3 = SolidPrimitive(); p3.type = SolidPrimitive.BOX; p3.dimensions = [0.004, 0.010, 0.004]
            pose3 = Pose(); pose3.position.x = x; pose3.position.y = y; pose3.position.z = z_floor + 0.056; pose3.orientation.w = 1.0
            obj.primitives.append(p3); obj.primitive_poses.append(pose3)

        obj.operation = CollisionObject.ADD
        self.collision_pub.publish(obj)
        time.sleep(0.01)
        
        if piece_type[0] == 'w':
            self.send_color_update(obj.id, 0.95, 0.94, 0.88)
        else:
            self.send_color_update(obj.id, 0.15, 0.15, 0.15)

    def remove_piece_collision_temporarily(self, obj_id):
        obj = CollisionObject()
        obj.id = obj_id
        obj.operation = CollisionObject.REMOVE
        self.collision_pub.publish(obj)

    # ✅ Işınlanmayı %100 çözen Frame ID ve Full PlanningScene Diff Metodu
    def attach_piece_to_gripper(self, piece_id):
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.robot_state.is_diff = True
        
        att_obj = AttachedCollisionObject()
        att_obj.link_name = "link_eef"
        att_obj.object.id = piece_id
        att_obj.object.header.frame_id = "link_base" # RViz'in taşı haritada kaybetmemesi için zorunlu
        att_obj.object.operation = CollisionObject.ADD
        
        att_obj.touch_links = [
            'left_finger', 'right_finger', 
            'left_inner_knuckle', 'right_inner_knuckle', 
            'left_outer_knuckle', 'right_outer_knuckle', 
            'xarm_gripper_base_link', 'link5', 'link6', 'link_eef'
        ]
        
        scene_msg.robot_state.attached_collision_objects.append(att_obj)
        self.scene_pub.publish(scene_msg)
        time.sleep(0.2)

    def detach_piece_from_gripper(self, piece_id):
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.robot_state.is_diff = True
        
        detach_obj = AttachedCollisionObject()
        detach_obj.link_name = "link_eef"
        detach_obj.object.id = piece_id
        detach_obj.object.header.frame_id = "link_base"
        detach_obj.object.operation = CollisionObject.REMOVE
        
        scene_msg.robot_state.attached_collision_objects.append(detach_obj)
        self.scene_pub.publish(scene_msg)
        time.sleep(0.2)

    def setup_initial_rviz_scene(self):
        columns = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
        print("🧹 Eski sahne tamamen temizleniyor...")
        
        for r in range(8):
            for c in range(8):
                sq_name = f"piece_{columns[c]}{8 - r}"
                self.detach_piece_from_gripper(sq_name)
                self.remove_piece_collision_temporarily(sq_name)
                self.remove_piece_collision_temporarily(f"square_{columns[c]}{8 - r}")
                
        for i in range(32):
            self.detach_piece_from_gripper(f"dead_white_{i}")
            self.detach_piece_from_gripper(f"dead_black_{i}")
            self.remove_piece_collision_temporarily(f"dead_white_{i}")
            self.remove_piece_collision_temporarily(f"dead_black_{i}")
            
        self.remove_piece_collision_temporarily("graveyard_left")
        self.remove_piece_collision_temporarily("graveyard_right")
        time.sleep(0.1)

        # ✅ Uçma Problemi Çözüldü: Masa kalınlığı ve zemin tekrar eski hizasında (Z = 0.0125)
        g_left = CollisionObject()
        g_left.header.frame_id = "link_base"
        g_left.id = "graveyard_left"
        p_l = SolidPrimitive()
        p_l.type = SolidPrimitive.BOX
        p_l.dimensions = [0.36, 0.08, 0.005]
        pos_l = Pose()
        pos_l.position.x = X_BOARD_START + (3.5 * SQUARE_SIZE)
        pos_l.position.y = Y_BOARD_START - 0.075
        pos_l.position.z = 0.0125
        pos_l.orientation.w = 1.0
        g_left.primitives.append(p_l)
        g_left.primitive_poses.append(pos_l)
        g_left.operation = CollisionObject.ADD
        self.collision_pub.publish(g_left)
        time.sleep(0.01)
        self.send_color_update(g_left.id, 0.25, 0.20, 0.15)

        g_right = CollisionObject()
        g_right.header.frame_id = "link_base"
        g_right.id = "graveyard_right"
        p_r = SolidPrimitive()
        p_r.type = SolidPrimitive.BOX
        p_r.dimensions = [0.36, 0.08, 0.005]
        pos_r = Pose()
        pos_r.position.x = X_BOARD_START + (3.5 * SQUARE_SIZE)
        pos_r.position.y = Y_BOARD_START + (SQUARE_SIZE * 8) + 0.035
        pos_r.position.z = 0.0125
        pos_r.orientation.w = 1.0
        g_right.primitives.append(p_r)
        g_right.primitive_poses.append(pos_r)
        g_right.operation = CollisionObject.ADD
        self.collision_pub.publish(g_right)
        time.sleep(0.01)
        self.send_color_update(g_right.id, 0.25, 0.20, 0.15)

        print("🏁 Çizgili Satranç Tahtası Çiziliyor...")
        for r in range(8):
            for c in range(8):
                sq_name = f"{columns[c]}{8 - r}"
                xyz = self.coord_map[sq_name]
                sq_obj = CollisionObject()
                sq_obj.header.frame_id = "link_base"
                sq_obj.id = f"square_{sq_name}"
                p = SolidPrimitive()
                p.type = SolidPrimitive.BOX
                p.dimensions = [SQUARE_SIZE-0.002, SQUARE_SIZE-0.002, 0.005]
                pose = Pose()
                pose.position.x = xyz['x']
                pose.position.y = xyz['y']
                pose.position.z = 0.015 - 0.0025 # Üst yüzey 0.015'e denk gelir, taşlarla sıfıra sıfır!
                pose.orientation.w = 1.0
                sq_obj.primitives.append(p)
                sq_obj.primitive_poses.append(pose)
                sq_obj.operation = CollisionObject.ADD
                self.collision_pub.publish(sq_obj)
                time.sleep(0.01)
                if (r + c) % 2 == 0:
                    self.send_color_update(sq_obj.id, 0.78, 0.62, 0.44)
                else:
                    self.send_color_update(sq_obj.id, 0.46, 0.28, 0.14)

        time.sleep(0.1)
        for r in range(8):
            for c in range(8):
                piece = self.gs.board[r][c]
                if piece != "--":
                    sq_name = f"{columns[c]}{8 - r}"
                    xyz = self.coord_map[sq_name]
                    self.spawn_advanced_piece(f"piece_{sq_name}", xyz['x'], xyz['y'], piece)

        for i, p_type in enumerate(self.captured_white_pieces):
            gx, gy = self.get_graveyard_coords_by_index(i, True)
            self.spawn_advanced_piece(f"dead_white_{i}", gx, gy, p_type)

        for i, p_type in enumerate(self.captured_black_pieces):
            gx, gy = self.get_graveyard_coords_by_index(i, False)
            self.spawn_advanced_piece(f"dead_black_{i}", gx, gy, p_type)

    def send_gripper_command(self, action_mode):
        target_pos = 0.85 if action_mode == "CLOSE" else 0.40
        req = GripperMove.Request()
        req.pos = 0.0 if action_mode == "CLOSE" else 400.0
        if self.gripper_client.wait_for_service(timeout_sec=0.02):
            self.gripper_client.call_async(req)
            
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['drive_joint']
        point = JointTrajectoryPoint()
        point.positions = [float(target_pos)]
        point.time_from_start.nanosec = 250000000 
        traj_msg.points.append(point)
        self.gripper_trajectory_pub.publish(traj_msg)
        time.sleep(0.5)

    def send_arm_to_xyz(self, x, y, z):
        req = PlanPose.Request()
        req.target.position.x = float(x)
        req.target.position.y = float(y)
        req.target.position.z = float(z)
        
        req.target.orientation.x = 1.0
        req.target.orientation.y = 0.0
        req.target.orientation.z = 0.0
        req.target.orientation.w = 0.0
        
        if not self.plan_client.wait_for_service(timeout_sec=2.0):
            return
            
        future = self.plan_client.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.01)
            
        res = future.result()
        if res is not None and res.success:
            exec_req = PlanExec.Request()
            exec_req.wait = True
            future_exec = self.exec_client.call_async(exec_req)
            while rclpy.ok() and not future_exec.done():
                time.sleep(0.01)
            time.sleep(0.15)
        else:
            self.get_logger().warn(f"⚠️ KİNEMATİK HATA VEYA ÇARPIŞMA: x:{x:.3f}, y:{y:.3f}, z:{z:.3f} hedefine ulaşılamadı!")

    def execute_pro_pick_and_place(self, start_x, start_y, dest_x, dest_y, piece_id, piece_type):
        z_grab_dynamic = self.get_grab_height(piece_type)
        
        self.send_arm_to_xyz(start_x, start_y, self.z_safe)
        self.send_gripper_command("OPEN")
        self.send_arm_to_xyz(start_x, start_y, z_grab_dynamic)
        
        self.send_gripper_command("CLOSE") 
        self.attach_piece_to_gripper(piece_id)
        
        self.send_arm_to_xyz(start_x, start_y, self.z_safe)
        self.send_arm_to_xyz(dest_x, dest_y, self.z_safe)
        self.send_arm_to_xyz(dest_x, dest_y, z_grab_dynamic)
        
        self.detach_piece_from_gripper(piece_id)
        self.send_gripper_command("OPEN") 
        
        self.send_arm_to_xyz(dest_x, dest_y, self.z_safe)

    def execute_robot_move(self, move_str, piece_type, captured_type):
        start_sq = move_str[:2]
        end_sq = move_str[2:]
        start_xyz = self.coord_map[start_sq]
        end_xyz = self.coord_map[end_sq]

        if captured_type != "--":
            print(f"🎯 Rakip {end_sq} karesindeki {captured_type} taşı mezarlığa ayıklanıyor...")
            if captured_type[0] == 'w':
                gx, gy = self.get_graveyard_coords_by_index(len(self.captured_white_pieces), True)
            else:
                gx, gy = self.get_graveyard_coords_by_index(len(self.captured_black_pieces), False)
            
            self.execute_pro_pick_and_place(end_xyz['x'], end_xyz['y'], gx, gy, f"piece_{end_sq}", captured_type)
            
            self.remove_piece_collision_temporarily(f"piece_{end_sq}")
            if captured_type[0] == 'w':
                dead_id = f"dead_white_{len(self.captured_white_pieces)}"
                self.spawn_advanced_piece(dead_id, gx, gy, captured_type)
                self.captured_white_pieces.append(captured_type)
            else:
                dead_id = f"dead_black_{len(self.captured_black_pieces)}"
                self.spawn_advanced_piece(dead_id, gx, gy, captured_type)
                self.captured_black_pieces.append(captured_type)

        print(f"🦾 Kendi taşını {start_sq} -> {end_sq} konumuna götürülüyor.")
        self.execute_pro_pick_and_place(start_xyz['x'], start_xyz['y'], end_xyz['x'], end_xyz['y'], f"piece_{start_sq}", piece_type)
        
        self.remove_piece_collision_temporarily(f"piece_{start_sq}")
        self.spawn_advanced_piece(f"piece_{end_sq}", end_xyz['x'], end_xyz['y'], piece_type)
        
        print("⚙️ [STATE MACHINE]: HOME -> Çevrim hatasız bitti, kol referans noktasına çekiliyor.")
        self.send_arm_to_xyz(self.x_home, self.y_home, self.z_home)

    def sync_piece_movement_in_rviz_for_human(self, start_sq, end_sq, piece_type, captured_type):
        if captured_type != "--":
            if captured_type[0] == 'w':
                gx, gy = self.get_graveyard_coords_by_index(len(self.captured_white_pieces), True)
                dead_id = f"dead_white_{len(self.captured_white_pieces)}"
                self.spawn_advanced_piece(dead_id, gx, gy, captured_type)
                self.captured_white_pieces.append(captured_type)
            else:
                gx, gy = self.get_graveyard_coords_by_index(len(self.captured_black_pieces), False)
                dead_id = f"dead_black_{len(self.captured_black_pieces)}"
                self.spawn_advanced_piece(dead_id, gx, gy, captured_type)
                self.captured_black_pieces.append(captured_type)

        self.remove_piece_collision_temporarily(f"piece_{start_sq}")
        if captured_type != "--":
            self.remove_piece_collision_temporarily(f"piece_{end_sq}")
        time.sleep(0.01)
        
        end_xyz = self.coord_map[end_sq]
        self.spawn_advanced_piece(f"piece_{end_sq}", end_xyz['x'], end_xyz['y'], piece_type)

    def print_board_terminal(self):
        print("\n  a  b  c  d  e  f  g  h")
        for i, row in enumerate(self.gs.board):
            row_str = f"{8-i} "
            for piece in row:
                if piece == "--":
                    row_str += " . "
                else:
                    row_str += f"{piece} "
            print(row_str)
        print("  a  b  c  d  e  f  g  h\n")

    def play_game_loop(self):
        time.sleep(1.5)
        self.send_arm_to_xyz(self.x_home, self.y_home, self.z_home)
        self.setup_initial_rviz_scene()
        print("⚡ Satranç Maçı Başladı! Sen BEYAZSIN, Robot SİYAH (Bot Dibinde).")
        
        while rclpy.ok():
            self.print_board_terminal()
            valid_moves = self.gs.get_valid_moves()
            if self.gs.checkmate:
                print("🏁 OYUN BİTTİ: MAT!")
                break
            if self.gs.stalemate:
                print("🏁 OYUN BİTTİ: PAT!")
                break

            if self.gs.white_to_move:
                user_input = input("Sıra Sende kanka (Örn: e2e4): ").strip()
                move_found = False
                for m in valid_moves:
                    if m.get_uci() == user_input:
                        p_type = self.gs.board[m.start_row][m.start_col]
                        captured_type = self.gs.board[m.end_row][m.end_col]
                        
                        self.execute_robot_move(m.get_uci(), p_type, captured_type)
                        self.gs.make_move(m)
                        
                        move_found = True
                        break
                if not move_found:
                    print("❌ Geçersiz hamle girdin kanka, kuralları kontrol et.")
                    continue
            else:
                print("🧠 Robot düşünüyor (Senin Hakiki Arama Algoritman Aktif)...")
                gs_clone = self.gs.clone()
                clone_valid_moves = gs_clone.get_valid_moves()
                best_move, score, nodes, max_depth = self.ai.find_best_move_smart(gs_clone, clone_valid_moves, 4.0)
                
                if best_move:
                    move_uci = best_move.get_uci()
                    p_type = self.gs.board[best_move.start_row][best_move.start_col]
                    captured_type = self.gs.board[best_move.end_row][best_move.end_col]
                    
                    self.execute_robot_move(move_uci, p_type, captured_type)
                    self.gs.make_move(best_move)
                    
                    print(f"🤖 Robotun Hamlesi: {best_move.get_chess_notation()}")
                else:
                    break

def main(args=None):
    rclpy.init(args=args)
    node = ChessBrainNode()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        node.play_game_loop() 
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
