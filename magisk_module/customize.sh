#!/system/bin/sh

set_perm_recursive $MODPATH 0 0 0755 0644
set_perm_recursive $MODPATH/service.sh 0 0 0755 0777
set_perm_recursive $MODPATH/post-fs-data.sh 0 0 0755 0777
set_perm_recursive $MODPATH/system/bin/lmkd 0 0 0755 0755 u:object_r:lmkd_exec:s0

echo "*******************************"
echo "   EMPModules Installed!   "
echo "*******************************"
