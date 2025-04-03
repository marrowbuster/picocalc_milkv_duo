#!/usr/bin/bash
cd /home/luckfox/picocalc_kbd-driver/
make
cp -f picocalc_kbd.ko /home/luckfox/luckfox-lyra-sdk-250311/buildroot/board/rockchip/rk3506/picocalc-overlay/usr/lib

cd /home/luckfox/picocalc_lcd_luckfox_lyra
make
cp -f ili9488_fb.ko /home/luckfox/luckfox-lyra-sdk-250311/buildroot/board/rockchip/rk3506/picocalc-overlay/usr/lib
