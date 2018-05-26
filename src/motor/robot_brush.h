#ifndef ROBOT_BRUSH_H
#define ROBOT_BRUSH_H

void robot_brush_update(void);
void robot_sidebrush_vols_set(u16 speed);
void robot_midbrush_vols_set(u16 speed);
void robot_brush_init(void);
extern int calc_cur_duty(int16_t voltage);
void robot_side_brush_adjust_set(u16 speed);
void robot_mid_brush_adjust_set(u16 speed);
#endif