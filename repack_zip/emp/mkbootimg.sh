#!/sbin/sh
#
# mkboot script helper
# Base of Anykernel by osm0sis (https://github.com/osm0sis).
# rainforce279 @ 17-12-2017

cmdline=`cat /tmp/emp/boot.img-cmdline`;
board=`cat /tmp/emp/boot.img-board`;
base=`cat /tmp/emp/boot.img-base`;
pagesize=`cat /tmp/emp/boot.img-pagesize`;
kerneloff=`cat /tmp/emp/boot.img-kerneloff`;
ramdiskoff=`cat /tmp/emp/boot.img-ramdiskoff`;
tagsoff=`cat /tmp/emp/boot.img-tagsoff`;
secondoff=`cat /tmp/emp/boot.img-secondoff`;

if [ -f /tmp/emp/boot.img-hash ]; then
	hashfck=`cat /tmp/emp/boot.img-hash`;
else
	hashfck=sha1;
fi;

if [ -f /tmp/emp/boot.img-osversion ]; then
	osver=`cat /tmp/emp/boot.img-osversion`;
else
	osver=7.1.2;
fi;

if [ -f /tmp/emp/boot.img-oslevel ]; then
	osvel=`cat /tmp/emp/boot.img-oslevel`;
else
	osvel=2018-05;
fi;

/tmp/emp/mkbootimg --kernel /tmp/emp/zImage --ramdisk /tmp/emp/boot.img-ramdisk.gz --cmdline "$cmdline" --board "$board" --base $base --pagesize $pagesize --kernel_offset $kerneloff --ramdisk_offset $ramdiskoff --second_offset $secondoff --tags_offset "$tagsoff" --os_version "$osver" --os_patch_level "$osvel" --hash "$hashfck" --output /tmp/emp/newboot.img;

