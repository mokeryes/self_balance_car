set remotetimeout 100
target extended-remote :3333
set remote hardware-watchpoint-limit 2
mon reset halt
flushregs
thb app_main
c
tui enable
