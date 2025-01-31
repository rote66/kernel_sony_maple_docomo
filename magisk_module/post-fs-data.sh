#!/system/bin/sh
MODDIR=${0%/*}

# set permission
chown system:system /proc/pressure/memory
chmod 0644 /proc/pressure/memory

# modify sepolicy rules
magiskpolicy --live "type proc_pressure_cpu { fs_type proc_type}"
magiskpolicy --live "type proc_pressure_io { fs_type proc_type}"
magiskpolicy --live "type proc_pressure_mem { fs_type proc_type}"
magiskpolicy --live "genfscon proc /pressure/cpu u:object_r:proc_pressure_cpu:s0"
magiskpolicy --live "genfscon proc /pressure/io u:object_r:proc_pressure_io:s0"
magiskpolicy --live "genfscon proc /pressure/memory u:object_r:proc_pressure_mem:s0"
magiskpolicy --live "allow lmkd proc_vmstat file { read open }"
magiskpolicy --live "allow lmkd proc_pressure_cpu file { read open }"
magiskpolicy --live "allow lmkd proc_pressure_io file { read open }"
magiskpolicy --live "allow lmkd proc_pressure_mem file { read open write }"
restorecon -v /proc/pressure/cpu
restorecon -v /proc/pressure/io
restorecon -v /proc/pressure/memory

# All Done
echo "EMP KERNEL Post-fs-data.sh Complete" >> /dev/kmsg
