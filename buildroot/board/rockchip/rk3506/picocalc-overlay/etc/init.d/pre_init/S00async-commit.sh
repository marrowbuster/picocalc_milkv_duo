#!/bin/sh
### BEGIN INIT INFO
# Provides:       async-commit
# Required-Start:
# Required-Stop:
# Default-Start:  S
# Default-Stop:
# Description:    Enable ASYNC_COMMIT for Rockchip BSP kernel > 4.4
### END INIT INFO

case "$1" in
	start)
#		/usr/bin/async-commit 2>&1 >/dev/null ;;
                ;;
	stop)
		;;
	restart|reload)
		;;
	*)
		echo "Usage: $0 {start|stop|restart}"
		exit 1
esac

exit $?
