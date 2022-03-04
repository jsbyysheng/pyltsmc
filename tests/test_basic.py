import build.pyltsmc as pyltsmc

pyltsmc.smc_set_connect_timeout(5000)
pyltsmc.smc_board_init(0, 2, '192.168.15.111', 115200)
print(pyltsmc.smc_get_release_version(0))
# print(pyltsmc.smc_get_total_axes(0))
# print(pyltsmc.smc_get_debug_mode())
