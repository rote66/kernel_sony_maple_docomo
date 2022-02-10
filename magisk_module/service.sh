#!/system/bin/sh
MODDIR=${0%/*}

# Optimized For Kernel

# Env
zrambw_dir=/data/per_boot
zrambw=/data/per_boot/zram_swap
loop_dir=/dev/block/loop0
zero=/dev/zero
# Size MB
zrambw_size=1024

# Start
echo "Service.sh start working" >> /dev/kmsg

# Force Loaded Wlan Driver
sleep 5
insmod /system/lib/modules/qca_cld3_wlan.ko

# Enable Zram Writeback Loop Device 
sleep 5
if [ ! -d $zrambw_dir ]; then mkdir $zrambw_dir; fi
if [ ! -f $zrambw ]; then dd if=$zero of=$zrambw bs=1024 count=1024*$zrambw_size; fi
losetup $loop_dir $zrambw
if [ $? -ne 0 ];then echo "mount zram backwrite loop device error !" >> /dev/kmsg; fi

# Disable Zram
swapoff /dev/block/zram0

# Set Zram
sleep 5
echo 1 > /sys/block/zram0/reset
# Wait
sleep 1
echo "/dev/block/loop0" > /sys/block/zram0/backing_dev
echo "lzo-rle" > /sys/block/zram0/comp_algorithm
echo 2202009600 > /sys/block/zram0/disksize
mkswap /dev/block/zram0
swapon /dev/block/zram0

# Set Zram Backwrite nodes permission
sleep 1
chown system:system /sys/block/zram0/idle
chown system:system /sys/block/zram0/writeback
chown system:system /sys/block/zram0/bd_stat
chown system:system /sys/block/zram0/backing_dev

# All Done
echo "Service.sh Complete" >> /dev/kmsg
