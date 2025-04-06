Require hardware modification

    1.cut the metal pin of GPIO4_B3 on luckfox lyra
    2.solder a wire from GP2 to GP27(PWM_R) on PicoCalc

Now RMIO_12 can be used to output audio via hardware pwm.

Device Tree

    / {
            compatible = "rockchip,rk3506";

            picocalc_snd {
                    status = "okay";
                    compatible = "fsl,picocalc-snd-pwm";
                    pinctrl-names = "default";
                    pwm-names = "pwm-snd-left";
                    pwms = <&pwm0_4ch_0 0 25000 0>;
            };
    };

    &pwm0_4ch_0 {
            pinctrl-names = "active";
            pinctrl-0 = <&rm_io12_pwm0_ch0>;
            status = "okay";
            assigned-clocks = <&cru CLK_PWM0>;
            assigned-clock-rates = <100000000>;
    };

/etc/asound.conf

    defaults.pcm.card 0
    defaults.ctl.card 0
