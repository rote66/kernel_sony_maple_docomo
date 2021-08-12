#!/system/bin/sh
echo "*******************************"
echo "   EMPModules Installed!   "
echo "*******************************"

set_perm_recursive $MODPATH 0 0 0755 0644
set_perm_recursive $MODPATH/service.sh 0 0 0755 0777