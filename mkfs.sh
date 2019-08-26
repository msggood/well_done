#!/bin/sh
genext2fs -b 8192 -d rootfs rootfs.ramdisk 
gzip -9 rootfs.ramdisk 
mkimage -A arm -O linux -T ramdisk -C gzip -n "Root Filesystem" -d rootfs.ramdisk.gz /tftpboot/uramdisk.image.gz
rm rootfs.ramdisk.gz
