
# FRC 2025 9231 Robot Kodu
Haydarpaşa Lisesi robotik takımımı Haydarpaşa Panthers'in 2025 Reefscape sezonuna ait robotun kodu. Bu sezonun temasında görevimiz mercan(içi boş beyaz boru şeklinde) adı verilen oyun parçalarını resif üzerindeki belli yüksekliklere takmak, alg(büyük mavi top) adı verilen oyun parçalarını resif üzerinden alıp puana dönüştürmek ve tırmanmak. 

![Logo](https://avatars.githubusercontent.com/u/162712686?s=100&u=d0ea36e11b59c011f9e8804e3c5b683a33d4264f&v=4)

    
## Belgelendirme

Takımın sürdürülebilirliği ve yazılım kaynakları için bu sene yazılım tarafında yapılanların ayrıntılı [kılavuz](https://panthers.gitbook.io/9231-2025-code-docs) (Sezondan sonra güncellenecek)

  
## Görüntüler


### Robot Tasarımı

### Simulasyonda robot

### Gerçek Robot

### Yarışmada Robot

### Elastic Dashboad Düzeni


  
## Otonom Rutini


## Kontrolcü buton atamaları

### Kullanılan Teknolojiler
Robotun kodunu yazarken işimizi kolaylaştıracak birçok eklenti, kütüphane ve araç kullandık.  
### Kütüphaneler
- [Advantagekit](https://docs.advantagekit.org) FRC robotlarında veri kaydı ve analiz yapmak için kullanılan bir kütüphanedir. Gelişmiş hata ayıklama ve maç analizi sağlar.
- [Pathplanner](https://pathplanner.dev) FRC robotları için kolayca otonomu planlamamızı sağlar.
- [Phoenix 6](https://v6.docs.ctr-electronics.com/en/latest/index.html) KrakenX60 motorları kontrol etmek için gerekli olan kütüphane. Talon FX motorları ile uyumlu çalışır.
- [photonlib](https://docs.photonvision.org/en/latest/index.html) Limelight'ın simulasyon desteği olmadığı için kameraları simüle etmek için kullandık. Kameraların sahadaki hedef verilerini simüle eder.
- [RevLib](https://docs.revrobotics.com/revlib) Neo motorları kontrol etmek için gerekli olan kütüphane. REV Robotics motor kontrol cihazları ve sensörleri ile uyumludur.

### Visual Studio Code Eklentileri
- [Error Lens](https://marketplace.visualstudio.com/items?itemName=usernamehw.errorlens) Hata mesajlarını ve uyarıları satır içinde göstererek hata ayıklamayı kolaylaştırır.
- [Gradle for Java](https://marketplace.visualstudio.com/items?itemName=vscjava.vscode-gradle) Java projelerinde Gradle görevlerini yönetmek için kullanılır.
- [Markdown Preview Enhanced](https://marketplace.visualstudio.com/items?itemName=shd101wyy.markdown-preview-enhanced) Markdown dosyalarını düzenlerken önizleme yapma imkanı sunar.
- [Project Manager](https://marketplace.visualstudio.com/items?itemName=alefragnani.project-manager) Proje yönetimini kolaylaştırarak farklı projeler arasında hızlı geçiş yapmayı sağlar.
- [Spotless Gradle](https://marketplace.visualstudio.com/items?itemName=richardwillis.vscode-spotless-gradle) Kod formatlama ve temizleme işlemleri için kullanılır.
- [Todo Tree](https://marketplace.visualstudio.com/items?itemName=Gruntfuggly.todo-tree) Kod içinde TODO notlarını otomatik olarak algılar ve listeler.

### Uygulamalar
- [2025 WPILIB VS Code](https://github.com/wpilibsuite/allwpilib) FRC robotları için geliştirme ortamı sağlar.
- [Pathplanner](https://github.com/mjansen4857/pathplanner) FRC robotları için yörünge oluşturma ve planlama aracı.
- [Advantage Scope](https://docs.advantagescope.org) Gerçek zamanlı olarak robot verilerini izlemek ve analiz etmek için kullanılır.
- [Elastic Dashboard](https://frc-elastic.gitbook.io/docs) Robot verilerini görselleştirmek ve analiz etmek için kullanılır.
- [REV Hardware Client](https://www.revrobotics.com/rev-hardware-client/) REV Robotics donanımını yapılandırmak ve kontrol etmek için kullanılır.
- [Phoenix Tuner X](https://v6.docs.ctr-electronics.com/en/stable/software/tuner-x.html) Motor denetleyicilerini ve sensörleri yapılandırmak için kullanılır.
- [Limelight Hardware Manager](https://www.limelightvision.io/) Limelight kamera ayarlarını yapılandırmak ve izlemek için kullanılır.
- [Touch Server](https://github.com/62Bytes/Touch-Server) Xbox kontrolcüsü olarak telefonu kullanmayı sağlar.
- [Balena Etcher](https://etcher.balena.io/) Bir cihaza yazılım yazdırmayı sağlar. Biz Roborio'yu ve Limelight'ı güncellemek için kullandık.
- [Network Assistant Tool](https://frc-radio.vivid-hosting.net/overview/programming-your-radio-at-home/network-assistant-tool) Vivid Hosting tarafından sağlanan yeni FRC radiolarını konfigüre etmek için kullanılır.
- [readme.so](https://readme.so/) Bu readME dosyasını oluşturmak için kullandık.

## Özellikler
- Bütün stageler'e mercan koyabilme
- Bütün stageler'den alg alabilme
- Algleri işleyebilme
- Hızlı cycle
- Full CTRE swerve
- Resife otomatik pozisyonlanma
- PID ve Motion Magic ile pozisyon kontrolü
- CommandBased kod yapısı
- AdvantageScope custom model konfigürasyonu
- Advantage scope ile tam simulasyon desteği
- Asansör simulasyonu
- Pathplanner ile otonom
- Hardware Abstraction kod yapısı
- Elastic Dasboard ile driver station
- SysID ile asansör ve drivetrain karakterizasyonu
- FaultReporters
- Detaylı loglar 
- Optimizasyon


  
## Çıkarılan Dersler

- Bu sene en zorlandığımız şey radio'yu konfigüre etmek oldu. Bu yüzden asla radio işini sona bırakmıyoruz.
- Limelight'ı da kalibre etmemiz gerektiğini öğrendik.
- İlk defa simulasyonda kendi robotumuzu simüle ettik. Bu yüzden biraz sancılıydı. Tam olarak edemedik ama genel olarak çalışıyor. Bunu sezondan önce öğrenmekte büyük fayda var.
- Roborio'yu her sezon güncellemeyi unutma!
- Limelight'ı her sezon güncellemeyi unutma!
- Radio'yu her sezon güncellemeyi unutma!
- Kodu commitlerken bütün yapılanları tek bir committe gönderme düzgünce ayır ve best practice yöntemleri kullan. Düzgün commit mesajları yaz. Yoksa daha sonra versiyonu geriye döndürmek istersen baş ağrıtabilir.
- Maple-sim ve superstructure kullanma planımız vardı. Maple-sim'i çalıştıramadık superstructure'u da işimize yarayacak bir durum olmadığı için kodunu sildik. Maple-sim entegrasyonu kesin sağla. Gerekiyorsa da superstructure kullan.
- Oynayan eklemlere yazılımsal limit ekle
- SysID düzenli olarak kullan.
- PID ayarlarken 0.1'den başla her seferinde iki katına çıkar hedef nokta etrafında sallanmaya başlayınca yavaş yavaş D değerini arttır.
- Koda bolca yorum ve açıklama ekle. 

  
## Bilgisayarınızda Çalıştırın

[WPILIB](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) indirin.

Projeyi klonlayın

```bash
  git clone https://github.com/FRC-Haydarpasa-Panthers-9231/9231_Reefscape_2025_RobotCode.git
```

2025 WPILIB VS Code üzerinden projeyi açın.

`CTRL + SHIFT + P` bastıktan sonra açılan pencereye `Simulate Robot Code` yazıp seçin.

Build almasını bekleyin.

`Enter` tuşuna basın ve simulasyonu çalıştırın.

 Simulasyonu `Teleoperated` moduna almayı ve kontrolcüyü seçmeyi unutmayın.







  
## Geliştiriciler ve Teşekkür

- [Haydarpaşa Panthers 9231](https://team9231.com) Takımdaki bütün üyelerin emeği için.

- [Hyperever](https://hyperever.com) Mentörlük destekleri için.

- [@dukeofsoftware](https://www.github.com/dukeofsoftware) 9231 Yazılım Kaptanına koda yaptığı katkılar için.

- [Mechanical Advantage 6328](https://github.com/mechanical-advantage) Kullanılan bazı util ve saha pozisyonu dosyaları için.

- [Windham Windup 3467](https://github.com/WHS-FRC-3467) Yazdıkları koddan ilham aldık.

- [The Junkyard Dogs 2106](https://www.team2106.org/) Led sistemi için.

- [FRC Discord Community](https://discord.gg/frc) Zorlandığımız kısımlarda topluluktan destek aldık.
## Ekler



  
## Geri Bildirim

Herhangi bir geri bildiriminiz varsa, lütfen kozanfurkanemre@gmail.com adresinden bize ulaşın.

  
## Lisans

[MIT](https://choosealicense.com/licenses/mit/)

  