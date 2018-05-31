#include <am_type.h>

enum {
    IR_SENDER_COUNT = 3,
    IR_RECEIVER_COUNT = 6,
};

enum {
    DOCK_IS_STEREO = TRUE,
    DOCK_IS_BLEB = FALSE,
};

u8 ir_map(int index);
float ir_receiver_distributions(int which);
float ir_receiver_angles_range(int which);
BOOLEAN ir_local_center_is_stereo();
u8 get_ir_local_center_index(u8 which);
BOOLEAN ir_local_is_bleb();
u8 get_ir_local_bleb_index();
u16 get_ir_dock_avoid_radius();
BOOLEAN ir_in_center(u8 ir);
BOOLEAN ir_no_signal(u8 ir);
BOOLEAN ir_match(u8 ir_in_map, u8 ir_receive);
u8 get_ir_with_beacons(u8 beacon);

BOOLEAN ir_good_distribution();

