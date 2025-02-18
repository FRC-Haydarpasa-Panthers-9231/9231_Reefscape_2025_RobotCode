// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.led;

// WPILib'in LED kontrolü ve yardımcı sınıfları
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** AddressableLedStrip, robotun LED şeridini kontrol etmek için kullanılan bir alt sistemdir. */
public class SUB_LED extends SubsystemBase {
    private static SUB_LED instance;

    private final AddressableLED ledStrip; // LED şeridini temsil eden nesne
    private final AddressableLEDBuffer ledBuffer; // LED şeridinin renk verilerini tutan tampon
    private final int length; // LED şeridinin uzunluğu
    public boolean endgameAlert = false;
    public boolean lowBatteryAlert = false;

    // LED efektlerini temsil eden durumlar
    public enum LEDState {
        RAINBOW, // Gökkuşağı efekti
        GREEN, // Tüm LED'ler yeşil olur
        BLUE, // Tüm LED'ler mavi olur
        RED, // Tüm LED'ler kırmızı olur
        WHITE, // Tüm LED'ler beyaz olur
        ORANGE, // Tüm LED'ler turuncu olur
        FLAME, // Ateş efekti (rastgele parlaklık ve renk)
        ALLAINCE, // İttifak rengine göre kırmızı veya mavi
        TURK,
        OFF, // LED'ler kapalı
        SCORING_ALGEA,
        INTAKING_ALGEA,
        SCORING_CORAL,
        MOVING_ELEVATOR,
        LOW_VOLTAGE,
        ENDGAME_ALERT,
        HAS_ALGEA,
        HAS_CORAL,
        CLEARING_L2,
        CLEARING_L3
    }

    private LEDState state; // Mevcut LED durumu
    private int hue; // Gökkuşağı efekti için kullanılan renk tonu

    /**
     * AddressableLedStrip yapıcısı, LED şeridini başlatır.
     *
     * @param port LED şeridinin bağlı olduğu PWM portu
     * @param length LED şeridinin uzunluğu
     */
    public SUB_LED(int port, int length)
    {
        this.length = length;
        ledStrip = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(length);

        ledStrip.setLength(ledBuffer.getLength()); // LED tamponunun uzunluğunu ayarlar
        ledStrip.setData(ledBuffer); // Tampon verilerini LED şeridine uygular
        state = LEDState.RAINBOW; // Varsayılan LED durumu olarak gökkuşağı efekti belirlenir
        hue = 0; // Gökkuşağı efekti için başlangıç tonu
        ledStrip.start(); // LED şeridini çalıştırır
    }

    /**
     * periodic() metodu, WPILib tarafından her döngüde çağrılır. LED efektleri bu metot içinde
     * kontrol edilir.
     */
    @Override
    public void periodic()
    {
        if (endgameAlert) {
            flame();
        }

        if (DriverStation.isDisabled()) {
            if (lowBatteryAlert) {
                solid(Color.kOrange);
            } else {
                // Robot devre dışıysa, gökkuşağı efekti göster
                rainbow();
            }

        } else if (!endgameAlert) {
            // Robot aktifken, mevcut duruma göre LED efektini uygula
            switch (state) {
                case RAINBOW:
                    rainbow();
                    break;
                case GREEN:
                    solid(Color.kGreen);
                    break;
                case BLUE:
                    solid(Color.kBlue);
                    break;
                case RED:
                    solid(Color.kRed);
                    break;
                case WHITE:
                    solid(Color.kWhite);
                    break;
                case ORANGE:
                    solid(Color.kOrange);
                    break;
                case FLAME:
                    flame();
                    break;
                case ALLAINCE:
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        if (alliance.get() == Alliance.Red) {
                            solid(Color.kRed);
                        } else if (alliance.get() == Alliance.Blue) {
                            solid(Color.kBlue);
                        }
                    } else {
                        solid(Color.kOrange);
                    }
                    break;
                case TURK:
                    everyOther(Color.kBlue, Color.kRed);
                    break;
                case OFF:
                    solid(Color.kBlack); // LED'leri kapat

                    // TODO: RENK PATTERNI BELİRLE VE CONSTANTS DOSYASI İLE STATELERİ YAZ.
                case SCORING_ALGEA:

                    break;
                case INTAKING_ALGEA:

                    break;
                case SCORING_CORAL:

                    break;
                case MOVING_ELEVATOR:

                    break;
                case LOW_VOLTAGE:

                    break;
                case ENDGAME_ALERT:

                    break;
                case HAS_ALGEA:

                    break;
                case HAS_CORAL:

                    break;

                case CLEARING_L2:
                    break;

                case CLEARING_L3:
                    break;

            }
        }

        // LED tamponunu günceller
        ledStrip.setData(ledBuffer);
    }

    /**
     * LED şeridinin durumunu ayarlar.
     *
     * @param state Yeni LED durumu
     */
    public void setState(LEDState state)
    {
        this.state = state;
    }

    /** Gökkuşağı efekti uygular. */
    private void rainbow()
    {
        for (var i = 0; i < length; i++) {
            final var hue = (this.hue + (i * 180 / length)) % 180;
            ledBuffer.setHSV(i, hue, 255, 128);
        }
        this.hue += 3; // Renk tonunu değiştirerek gökkuşağı animasyonu oluştur
        this.hue %= 180;
    }

    /**
     * Tüm LED şeridine tek bir renk uygular.
     *
     * @param color Uygulanacak renk
     */
    private void solid(Color color)
    {
        for (var i = 0; i < length; i++) {
            ledBuffer.setLED(i, color);
        }
    }

    /**
     * LED'lere sırayla iki farklı renk uygular (örneğin, mavi ve kırmızı).
     *
     * @param color1 İlk renk
     * @param color2 İkinci renk
     */
    private void everyOther(Color color1, Color color2)
    {
        for (var i = 0; i < length; i++) {
            if (i % 2 == 0) {
                ledBuffer.setLED(i, color1);
            } else {
                ledBuffer.setLED(i, color2);
            }
        }
    }

    /**
     * LED şeridinin tamamına rastgele bir parlaklık ve renk uygular. Bu efekt ateş animasyonunu
     * simüle eder.
     */
    private void flame()
    {
        for (var i = 0; i < length; i++) {
            int brightness = (int) (Math.random() * 128) + 128; // Rastgele parlaklık (128-255
            // arası)
            int hue = (int) (Math.random() * 180); // Rastgele ton (0-180 arası)
            ledBuffer.setHSV(i, hue, 255, brightness);
        }
    }

    public static SUB_LED getInstance()
    {
        if (instance == null) {
            instance = new SUB_LED(LedConstants.kLedPort, LedConstants.kLedLength);
        }
        return instance;
    }
}
