Markdown
# Bitirme: Tam Otonom Mekatronik Satranç Sistemi ♟️🤖

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.10-blue)
![MoveIt 2](https://img.shields.io/badge/MoveIt_2-Enabled-orange)
![xArm6](https://img.shields.io/badge/Hardware-xArm6-green)

Yapay zeka karar çekirdeği (Minimax ve Alpha-Beta Budama) ile 6 serbestlik dereceli (6-DOF) endüstriyel robot kolunu (xArm6), ROS 2 ve MoveIt 2 altyapısı kullanarak entegre eden, kapalı çevrim tam otonom mekatronik satranç motoru.

## 🚀 Temel Özellikler
* **Özel Yapay Zeka Motoru:** Python ile kodlanmış; Negamax varyasyonu, Alpha-Beta budama, yinelemeli derinleşme (Iterative Deepening) ve JSON tabanlı açılış kütüphanesi (`beyin.json`) kullanan otonom karar mekanizması.
* **ROS 2 Dağıtık Düğüm Mimarisi:** DDS ağı üzerinden, yapay zeka karar düğümü (Node) ile donanım kontrolcüleri arasında asenkron (bloklanmayan) çoklu iş parçacıklı (Multithreading) haberleşme.
* **Akıllı Kavrama (Dinamik Z-Ekseni):** Mekanik çarpışmaları (devrilmeleri) önlemek amacıyla, robotun Z-ekseni (iniş derinliği) hedefini satranç taşının fiziksel boyuna göre (Örn: Piyon için 0.045m, Şah için 0.062m) algoritmik olarak ayarlaması.
* **Mekanik Çarpışma Önleme (Collision Avoidance):** RViz 2 simülasyonunda "Dijital İkiz" objelerine 1 mm'lik mekanik boşluk toleransı tanınması ve Mezarlık (Graveyard) dizilimi için modüler aritmetikle 6.5 cm'lik güvenli ofset koridoru oluşturulması.
* **Endüstriyel Operatör Paneli (HMI):** `PyQt6` kütüphanesiyle tasarlanmış; canlı yapay zeka telemetrisi, dinamik kazanma olasılığı hesaplaması ve PGN formatında operasyon kaydı (Data Logging) sunan grafiksel arayüz.

## 🛠️ Donanım ve Yazılım Altyapısı
* **Donanım:** xArm6 Robot Kolu, Motorize Paralel Tutucu (Gripper)
* **Ara Katman (Middleware):** ROS 2 (Humble), MoveIt 2 (KDL Kinematics)
* **Ana Programlama Dili:** Python 3.10
* **Görsel Arayüz (GUI):** PyQt6

## ⚙️ Kurulum ve Çalıştırma

1. **Projeyi bilgisayarınıza klonlayın:**
   ```bash
   git clone [https://github.com/AhmetBugra46/Yapay-zeka.git](https://github.com/AhmetBugra46/Yapay-zeka.git)
   cd Yapay-zeka
Gerekli Python kütüphanelerini yükleyin:

Bash
pip install -r requirements.txt
Operatör arayüzünü ve otonom ROS 2 düğümünü başlatın:

Bash
python3 src/chess_brain_node.py

Sistem Çalışma Görüntüsü

[4. hata) Yörünge planlama hatası.webm](https://github.com/user-attachments/assets/c4625c83-1c54-4a2b-9208-844198b1b68e)
[4. hata) Tahtadaki taşların kaybolma hatası.webm](https://github.com/user-attachments/assets/5a9510ea-7a43-4db0-a616-f926a3e8afe5)
[3. hata) Robot kol çakışma hatası.webm](https://github.com/user-attachments/assets/545d431f-d066-4a11-801c-3cc18f965aa5)
[3) Şah durumunda geçersiz hamle.webm](https://github.com/user-attachments/assets/a824344b-272e-4b97-b8c6-d93eb56d4f1c)
[2. hata) Kavramama,bozulma hatası.webm](https://github.com/user-attachments/assets/95dfb78d-9cd7-4a21-aeb8-c7a613d11616)
[2) Taş yeme durumu.webm](https://github.com/user-attachments/assets/3527e986-28d6-461c-8cee-658b7414d8ea)
[1. hata) Grip açmama, Işınlanma hatası.webm](https://github.com/user-attachments/assets/84bb972e-8fd9-4b6d-a934-e7728c9cd64c)

👨‍💻 Geliştirici
Ahmet Buğra KURTBOĞAN

Mekatronik Mühendisliği | Erciyes Üniversitesi
