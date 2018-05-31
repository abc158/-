#include "am-docking-utils.h"
#include "am-docking-sensors.h"
#include "am-docking-config.h"
#include <math.h>

float AM_math_angle_diff_f(float angle1, float angle2)
{
    float a = angle1 - angle2 + M_PI_F;
    /*if(a > 2*M_PI_F)
    {
        a = a - 2*M_PI_F*am_floorf(a/(2*M_PI_F));
    }
    else if(a<0)
    {
        a = a + 2*M_PI_F*(1+am_floorf(-a/(2*M_PI_F)));
    }*/
    while (a > 2*M_PI_F)
        a -= 2*M_PI_F;
    while (a < 0)
        a += 2*M_PI_F;

    return a - M_PI_F;
}

float AM_math_angle_add_f(float angle1, float angle2)
{
    return AM_math_angle_diff_f(angle1,-angle2);
}

struct {
    void (*AM_Pose2D_transform_xy)(const AM_Pose *pose, const AM_xy* xy, AM_xy *pose_xy);
    void (*turn_in_place)(am_real limit, int speed);
    int64_t (*rate_gyro_get_angle_int)(void);
    void (*robot_to_world)(const AM_Pose2D *cur_robot_pos, const AM_xy *sample_point, AM_xy *out);
    void (*turn_circle)(am_real limit, int speed);
    void (*irmap_mutex_init)();
    void* (*irmap_mutex_lock)();
    void (*irmap_mutex_unlock)(void *lock);

} _dock_api = { 0 };

void setup_dock_api() {
    memset(&_dock_api, 0, sizeof(_dock_api));
    get_dock_api((void*)&_dock_api, sizeof(_dock_api) / sizeof(void*));
}

void turn_in_place(am_real limit, int speed) {
    if (_dock_api.turn_in_place)
        _dock_api.turn_in_place(limit, speed);
}

void AM_Pose2D_transform_xy(const AM_Pose *pose, const AM_xy* xy, AM_xy *pose_xy) {
    if (_dock_api.AM_Pose2D_transform_xy)
        _dock_api.AM_Pose2D_transform_xy(pose, xy, pose_xy);
}

int64_t rate_gyro_get_angle_int(void) {
    if (_dock_api.rate_gyro_get_angle_int)
        return _dock_api.rate_gyro_get_angle_int();
    return 0;
}

void robot_to_world(const AM_Pose2D *cur_robot_pos, const AM_xy *sample_point, AM_xy *out) {
    if (_dock_api.robot_to_world)
        _dock_api.robot_to_world(cur_robot_pos, sample_point, out);
}

void turn_circle(am_real limit, int speed) {
    if (_dock_api.turn_circle)
        _dock_api.turn_circle(limit, speed);
}

void irmap_mutex_init() {
    if (_dock_api.irmap_mutex_init)
        _dock_api.irmap_mutex_init();
}
void *irmap_mutex_lock() {
    if (_dock_api.irmap_mutex_lock)
        return _dock_api.irmap_mutex_lock();
    return NULL;
}
void irmap_mutex_unlock(void *lock) {
    if (_dock_api.irmap_mutex_unlock)
        _dock_api.irmap_mutex_unlock(lock);
}


// tip check

// sdk <-> app
#define get_ir_distributions_in_robot ir_receiver_distributions
#define TURN_IN_PLACE AM_GO_TO_PLACE

#define IR_TIP_SIGNAL_COUNT IR_RECEIVER_COUNT
#define IR_DOCK_AREA_COUNT  IR_SENDER_COUNT
#define IR_DOCK_AREA_START  1

static am_boolean am_tip_check_avoid(int index) {
    return recently_avoid_receiver[index].current_state;;
}

static am_boolean am_tip_check_signal(int index) {
    return recently_receiver[index].current_state;
}

static am_boolean am_dock_area_signal(int area) {
    return recently_sender[area].current_state;
}

static am_boolean am_check_signal() {
    return recently_signal_short_term.current_state;
}

static am_boolean am_check_avoid_signal() {
    return recently_near_dock_2.current_state;
}

static am_boolean am_check_center_signal(){
    return recently_center.current_state;
}


// debug assist
#define AM_DOCKING_DEBUG
#ifdef AM_DOCKING_DEBUG
enum {
	DEBUG_DVRIER_GO = 0x1 << 0,
    DEBUG_TURN      = 0x1 << 1,
    DEBUG_DOCK_POSE = 0x1 << 2,
    DEBUG_TIP       = 0x1 << 3,
};

static U8 debug_mask = DEBUG_TIP;
#define dprintf(level_mask, msg...)  if(debug_mask&level_mask)\
	printf(msg)
#else
#define dprintf(msg...) {}
#endif


enum {
    TIP_SEARCH_AVOID = 0,
    TIP_SEARCH_SIGNAL,
};

#define CIRCLE_STEP (3)
#define CIRCLE_BITS (360/CIRCLE_STEP)
#define CIRCLE_BYTES ((CIRCLE_BITS+7)/8)
#define DOCKING_NEAR_SLOWEST_SPEED    (70)


// only '0' -> '1'
static void setbit(am_uint8 arr[], am_uint32 n) {
    int a = n / 8;
    int b = n % 8;
    arr[a] |= (1 << b);
}

static am_boolean checkbit(am_uint8 arr[], am_uint32 n) {
    int a = n / 8;
    int b = n % 8;
    return (arr[a] & (1 << b)) != 0;
}

// 0..CIRCLE_BITS, [begin, end)
static int calc_longest_zero(am_uint8 ring[], int bits, int* begin, int* end) {
    int beginning_one = -1;
    int ending_one = -1;
    int a = 0;
    int b = 0;
    int longest_size = 0;
    int last_one = 0;
    for (int i = 0; i < bits; ++i) {
        am_boolean is_set = (ring[a] & (1<<b)) != 0;
        if (is_set) {
            if (beginning_one < 0)
                beginning_one = i;
            ending_one = i;
            int size = i - (last_one+1);
            if (size > longest_size) {
                longest_size = size;
                *begin = (last_one+1);
                *end = i;
            }
            last_one = i;
        }
        if (++b == 8) {
            a++;
            b = 0;
        }
    }
    if (beginning_one == -1 && ending_one == -1) { // check all zero
        longest_size = bits;
        *begin = 0;
        *end = bits;
    } else if ((bits - 1) > ending_one) { // check ring
        int ring_zero_size = beginning_one + (bits - 1) - ending_one;
        if (ring_zero_size > longest_size) {
            longest_size = ring_zero_size;
            *begin = ending_one + 1;
            *end = beginning_one;
        }
    }

    return longest_size;
}

// 0..360, [begin, end)
static int calc_valid_range(am_uint8 ring[], int bits, int step, int* begin, int* end) {
    int zero_begin = -1;
    int zero_end = -1;
    int longest_zero = calc_longest_zero(ring, bits, &zero_begin, &zero_end);
    if (longest_zero == bits) { // no '1'
        *begin = *end = -1;
        return 0;
    } else {
        *begin = (zero_end==bits ? 0 : zero_end) * step;
        *end = (zero_begin==0 ? bits : zero_begin) * step;
        return (bits-longest_zero) * step;
    }
}

static int norm_360(int angle) {
    while (angle >= 360)
        angle -= 360;
    while (angle < 0)
        angle += 360;
    return angle;
}

static int get_set_count(am_uint8 arr[8]) {
    int count = 0;
    for (int i = 0; i < 8; ++i) {
        if (arr[i] != 0)
            count++;
    }
    return count;
}

static int get_set_count_1(am_uint8 arr) {
    int count = 0;
    for (int i = 0; i < 8; ++i) {
        if (arr&(0x01<<i))
            count++;
    }
    return count;
}


static float calc_avg_middle(float middles[8], am_uint8 is_valid[8]) {
    float first_middle = 0.f;
    am_boolean first_middle_valid = FALSE;
    float middle_avg_gain = 0.f;
    int count = 0;

    for (int i = 0; i < 8; ++i) {
        if (is_valid[i]) {
            if (!first_middle_valid) {
                first_middle = middles[i];
                first_middle_valid = TRUE;
            }
            middle_avg_gain += AM_math_angle_diff_f(middles[i], first_middle);
            count++;
        }
    }
    return count == 0 ? 0.f : AM_math_angle_add_f(first_middle, middle_avg_gain/count);;
}

static int calc_count_under(float middles[8], am_uint8 is_valid[8], float avg, int threshold, am_uint8 unders[8]) {
    int c = 0;
    memset(unders, 0, 8);
    for (int i = 0; i < IR_TIP_SIGNAL_COUNT; ++i) {
        if (is_valid[i]) {
            int diff = abs(R2D(AM_math_angle_diff_f(middles[i], avg)));
            if (diff < threshold) {
                unders[i] = 1;
                c++;
            }
        }
    }
    return c;
}

typedef am_boolean (*am_tip_check_func)(int index);

typedef struct {
    am_tip_check_func tip_check;
    int signal_area;
    am_boolean is_center;
    am_boolean is_avoid;
    am_boolean is_valid; // valid when has signal
    am_boolean stop_when_center;

    float dock_direction;
    float jamming_direction;
    am_boolean dock_direction_valid;
    am_boolean jamming_direction_valid;

    am_int64 orient;
    am_uint8 hitpoint[8][CIRCLE_BYTES]; // 15*8bit=120, 3deg/1bit

    AM_Pose2D curr;
    am_uint16 last_check;
    am_uint8 avoid_signal_chan_count;
	
} tip_search_info;
static tip_search_info _ts;

static am_boolean am_tip_reset(int which, am_boolean stop_when_center) {

    memset(&_ts, 0, sizeof(_ts));
    _ts.stop_when_center = stop_when_center;
    if (which == TIP_SEARCH_AVOID) {
        _ts.tip_check = am_tip_check_avoid;
    } else if (which == TIP_SEARCH_SIGNAL) {
        _ts.tip_check = am_tip_check_signal;
    } else {
        return FALSE;
    }
    _ts.orient = rate_gyro_get_angle_int();
    localization_get_global_pos(&_ts.curr);
    return TRUE;

}

static void am_tip_dump_hitpoint() {
    for (int i = 0; i < IR_TIP_SIGNAL_COUNT; ++i) {
        dprintf(DEBUG_TIP, "hitp[%d]: ", i);
        for (int j = 0; j < CIRCLE_BYTES; ++j) {
            dprintf(DEBUG_TIP, "%x ", _ts.hitpoint[i][j]);
        }
        dprintf(DEBUG_TIP, "\r\n");
    }
}

// include jamming judge
static void am_tip_calc_direction() {
    int ranges[8];
    float middles[8];
    am_uint8 is_valid[8];
    am_uint8 is_jamming[8];
    int count = 0;

    memset(is_valid, 0, 8);
    memset(is_jamming, 0, 8);

    for (int i = 0; i < IR_TIP_SIGNAL_COUNT; ++i) {
        if (ir_local_is_bleb() && (get_ir_local_bleb_index() == i)) // exclude bleb
            continue;
        int begin, end;
        int range = calc_valid_range(_ts.hitpoint[i], CIRCLE_BITS, CIRCLE_STEP, &begin, &end);
        if (range > 0) {
            ranges[i] = range;
            middles[i] = AM_math_angle_add_f((begin + range/2)*M_PI_F/180, get_ir_distributions_in_robot(i));
            dprintf(DEBUG_TIP, "range[%d](%d,%d),m=%d\r\n", i, range, R2D(ir_receiver_angles_range(i)), R2D(middles[i]));
            if (range > R2D(ir_receiver_angles_range(i))+30)
                is_jamming[i] = 1;
            else
                is_valid[i] = 1;
            count++;
        }
    }

    float avg_middle = calc_avg_middle(middles, is_valid);
    dprintf(DEBUG_TIP, "avg=%d\r\n", R2D(avg_middle));

    // check jamming from range
    am_uint8 unders[8];
    int count_under = calc_count_under(middles, is_valid, avg_middle, 30, unders);
    int count_valid = get_set_count(is_valid);
    dprintf(DEBUG_TIP, "valid=%d,under=%d,all=%d\r\n", count_valid, count_under, count);
    if (count_under > count/2) { // unders is valid
        for (int i = 0; i < IR_TIP_SIGNAL_COUNT; ++i) {
            if (is_valid[i] && !unders[i]) {
                is_jamming[i] = 1;
            }
            is_valid[i] = unders[i];
        }
    } else if (count_valid-count_under > count/2) { // unders is invalid
        for (int i = 0; i < IR_TIP_SIGNAL_COUNT; ++i) {
            if (is_valid[i] && unders[i]) {
                is_jamming[i] = 1;
            }
            is_valid[i] = is_valid[i] && !unders[i];
        }
    } else {
        dprintf(DEBUG_TIP, "fail c=%d,%d\r\n", count_valid, count_under);
    }
    dprintf(DEBUG_TIP, "valid/jam/und:\r\n");
    for (int i = 0; i < IR_TIP_SIGNAL_COUNT; ++i)
        dprintf(DEBUG_TIP, "%d ", is_valid[i]);
    dprintf(DEBUG_TIP, "\r\n");
    for (int i = 0; i < IR_TIP_SIGNAL_COUNT; ++i)
        dprintf(DEBUG_TIP, "%d ", is_jamming[i]);
    dprintf(DEBUG_TIP, "\r\n");
    for (int i = 0; i < IR_TIP_SIGNAL_COUNT; ++i)
        dprintf(DEBUG_TIP, "%d ", unders[i]);
    dprintf(DEBUG_TIP, "\r\n");

    _ts.dock_direction_valid = get_set_count(is_valid) > 0;
    if (_ts.dock_direction_valid)
        _ts.dock_direction = calc_avg_middle(middles, is_valid);
    _ts.jamming_direction_valid = get_set_count(is_jamming) > 0;
    if (_ts.jamming_direction_valid) {
        float jamming_avg = calc_avg_middle(middles, is_jamming);
        _ts.jamming_direction = AM_math_angle_add_f(jamming_avg, AM_math_angle_diff_f(jamming_avg, _ts.dock_direction));
    }

    // add curr
    _ts.dock_direction = AM_math_angle_add_f(_ts.curr.angle, _ts.dock_direction);
    _ts.jamming_direction = AM_math_angle_add_f(_ts.curr.angle, _ts.jamming_direction);


    dprintf(DEBUG_TIP, "dock=%d(v=%d,c=%d),jam=%d(v=%d,c=%d)\r\n",
        R2D(_ts.dock_direction), _ts.dock_direction_valid, get_set_count(is_valid),
        R2D(_ts.jamming_direction), _ts.jamming_direction_valid, get_set_count(is_jamming));

}

// [0..upper]
static int am_tip_calc_direction_patial_hitpoint(int index, int upper) {
    am_uint8 *hitpoint = &_ts.hitpoint[index][0];
    if (checkbit(hitpoint, 0) || checkbit(hitpoint, upper))
        return -1;
    int left = -1, right = -1;
    am_boolean prev = checkbit(hitpoint, 0);
    for (int i = 1; i <= upper; ++i) {
        am_boolean curr = checkbit(hitpoint, i);
        if (!prev && curr) {
            if (left > 0)
                return -1;
            left = i;
        }
        if (prev && !curr) {
            if (right > 0)
                return -1;
            right = i;
        }
        prev = curr;
    }
    if (left == -1 || right == -1)
        return -1;
    //printf("[%d] l/r=%d,%d\r\n", index, left, right);
    int range = (right - left) * CIRCLE_STEP;
    int range_config = R2D(ir_receiver_angles_range(index));
    if (range < range_config-15 || range > range_config+15) {
        //printf("[%d] range err %d,%d\r\n", index, range, range_config);
        return -1;
    }
    return (left + right) * CIRCLE_STEP / 2;
}

static int am_tip_get_rotate() {
    int rotate = (rate_gyro_get_angle_int() - _ts.orient);
    if (rotate >= 360)
        rotate -= 360;
    if (rotate < 0)
        rotate = 0;
    return rotate;
}

// without jamming judge
static am_boolean am_tip_check_patial_hitpoint() {
    int rotate = am_tip_get_rotate();
    if (rotate > _ts.last_check) {
        _ts.last_check += 15; // check partial each 15 deg
        printf("chk rot=%d\r\n", rotate);
        float direction[8];
        am_uint8 valid[8];
        int count = 0;
        memset(valid, 0, 8);
        for (int i = 0; i < IR_TIP_SIGNAL_COUNT; ++i) {
            int d = am_tip_calc_direction_patial_hitpoint(i, rotate / CIRCLE_STEP);
            if (d > 0) {
                direction[count] = AM_math_angle_add_f(d*M_PI_F/180, get_ir_distributions_in_robot(i));
                valid[count] = 1;
                count++;
            }
        }
        if (count > 1) {
            //printf("partial ok\r\n");
            for (int i = 0; i < IR_TIP_SIGNAL_COUNT; ++i) {
                if (valid[i])
                    printf("[%d] dir=%d\r\n", i, R2D(direction[i]));
            }
            _ts.dock_direction_valid = TRUE;
            _ts.dock_direction = AM_math_angle_add_f(_ts.curr.angle, calc_avg_middle(direction, valid));
            return TRUE;
        }
    }
    return FALSE;
}

static am_boolean am_tip_check() {
    am_uint8 avoid_signal_chan = 0;
    for (int i = 0; i < IR_TIP_SIGNAL_COUNT; ++i) {
        if(am_tip_check_avoid(i)) 
            avoid_signal_chan |= (0x1 << i);//avoid chan
        if (ir_local_is_bleb() && (get_ir_local_bleb_index() == i)) // exclude bleb
            continue;
        am_boolean tip_check_state = _ts.tip_check(i);
        if (tip_check_state) {
            int n = am_tip_get_rotate() / CIRCLE_STEP;
            setbit(_ts.hitpoint[i], n);
        }
    }
    for (int i = 0; i < IR_DOCK_AREA_COUNT; ++i){
        if (am_dock_area_signal(i)) {
            _ts.signal_area |= (0x1 << (i+IR_DOCK_AREA_START));
        }
    }
    if (am_check_avoid_signal()) {
        _ts.is_avoid = TRUE;
        _ts.avoid_signal_chan_count = get_set_count_1(avoid_signal_chan);
    }
    if (am_check_signal())
        _ts.is_valid = TRUE;
    if (am_check_center_signal()) {
        _ts.is_center = TRUE;
        if (_ts.stop_when_center)
            return FALSE;
    }

    if (ir_good_distribution()) {
        /*if (_ts.is_center && am_tip_get_rotate() > 100) {
            printf("center rotate\r\n");
            return FALSE;
        }*/
        am_uint32 timer_start = timer_ms();
        if (am_tip_check_patial_hitpoint()) {
            printf("partial cost %d\r\n", timer_ms()-timer_start);
            return FALSE;
        }
    }

    return TRUE;
}

static am_boolean am_tip_get_info() {
    am_tip_dump_hitpoint();
    if (!_ts.dock_direction_valid)
        am_tip_calc_direction();
    return _ts.is_valid;
}

am_boolean am_tip_search_info(am_boolean stop_when_center) {
    am_uint8 result;
    if (!am_tip_reset(TIP_SEARCH_SIGNAL, stop_when_center)) {
        return FALSE;
    }

    TURN_IN_PLACE(360,DOCKING_NEAR_SLOWEST_SPEED,DOCKING_NEAR_SLOWEST_SPEED,am_tip_check(),0,result);

    return am_tip_get_info();
}

void am_tip_get(void *info, int size) {
    int info_size = sizeof(_ts);
    memcpy(info, &_ts, size > info_size ? info_size : size);
}



// irmap

#define IR_HYPOTHESES_TIMES (8) //

static const float theta_step = 1.5f*M_PI_F/180;
static const float xy_step = 0.01f;

static float xy_float(am_int16 xy) {
    return xy * xy_step;
}

static float theta_float(am_int8 t) {
    return t * theta_step;
}

static am_int16 xy_int(float xy) {
    return xy / xy_step;
}

static am_int8 theta_int(float t) {
    return t / theta_step;
}

static int theta_degree(am_int8 t) {
    return t * 3 / 2;
}

static float IRPoint_distance(IRPoint* p1, IRPoint* p2) {
    float diff_x = xy_float(p1->x - p2->x);
    float diff_y = xy_float(p1->y - p2->y);
    return sqrt(diff_x * diff_x + diff_y * diff_y);
};
static float IRPoint_diff_angle(IRPoint* p1, IRPoint* p2) {
    return abs(theta_float(p1->theta - p2->theta));
}

static void IRTraj_init(IRTraj *traj) {
    traj->head = traj->tail = traj->count = 0;
    traj->capacity = IR_POSE_MAX_SIZE;
}

static void IRTraj_push_back(IRTraj *traj, IRPoint *p) {
    IRPoint *to_write = &traj->points[traj->tail];
    if (++traj->tail >= traj->capacity) {
        traj->tail = 0;
        if (traj->tail == traj->head) { // overflow
            if (++traj->head >= traj->capacity)
                traj->head = 0;
            traj->count--;
        }
    }
    *to_write = *p;
    traj->count++;
}

static int IRTraj_size(IRTraj *traj) {
    return traj->count;
}

static int IRTraj_capacity(IRTraj *traj) {
    return traj->capacity;
}

static void IRTraj_clear(IRTraj *traj) {
    IRTraj_init(traj);
}

// no guarantee for valid 'index'
static IRPoint *IRTraj_get(IRTraj *traj, int index) {
    return &traj->points[index];
}

static void IRTraj_copy(IRTraj *from, IRTraj *to) {
    memcpy(to, from, sizeof(IRTraj));
}

static void Hypotheses_reset(Hypotheses *h) {
    h->count = 0;
    h->capacity = IR_HYPOTHESES_EACH;
}

static int Hypotheses_size(Hypotheses *h) {
    return h->count;
}

static int Hypotheses_capacity(Hypotheses *h) {
    return h->capacity;
}

static void Hypotheses_push(Hypotheses *h, AM_Pose2D* model) {
    AM_Pose2D *to_write = &h->models[h->count];
    if (h->count >= h->capacity)
        return;
    *to_write = *model;
    h->count++;
}

// no guarantee for valid 'index'
static AM_Pose2D* Hypotheses_get(Hypotheses *h, int index) {
    return &h->models[index];
}

static float robot_radius = 0.15f;
static const float ir_receiver_range_min = 100 * M_PI_F / 180;  // add to dock theta
static const float ir_receiver_range_max = -100 * M_PI_F / 180; // add to dock theta


static am_boolean _ir_map_inited = FALSE;
static IRTraj *_traj = NULL;

static AM_Pose2D _dock_irmap = { 1., 2., M_PI_F * 6 / 4 };  // dock pose in irmap
static AM_Pose2D _dock_global = { 1., 1., M_PI_F * 5 / 4 };  // dock pose for test
static int _fence_count = 0;

void ir_map_init(IRTraj *traj) {
    if (_ir_map_inited)
        return;
    printf("ir_map_init\n\r");
    irmap_mutex_init();
    IRTraj_init(traj);
    _ir_map_inited = TRUE;
    _traj = traj;
}

static am_uint8 get_ir_with_pose(const AM_Pose2D* pose, const AM_Pose2D* dock) {
    // rotate by dock
    float angle = _dock_irmap.angle - dock->angle;
    float s = sin(angle);
    float c = cos(angle);
    float x = c*(pose->xy.x - dock->xy.x) - s*(pose->xy.y - dock->xy.y) + dock->xy.x;
    float y = s*(pose->xy.x - dock->xy.x) + c*(pose->xy.y - dock->xy.y) + dock->xy.y;
    x += _dock_irmap.xy.x - dock->xy.x;
    y += _dock_irmap.xy.y - dock->xy.y;
    if (x < 0 || x >= 2 || y < 0 || y >= 2)
        return 0;
    int irx = x * 10;
    int iry = (2 - y) * 10;
    return ir_map(iry * 20 + irx);
}

void ir_map_add_traj_pose(IRTraj *traj, AM_Pose2D* pose, unsigned char ir, int which) {
    //printf("add traj b=0x%x.\n\r", beacons);
    void *lock = irmap_mutex_lock();
    if (!_ir_map_inited) {
        irmap_mutex_unlock(lock);
        return;
    }
    AM_Pose2D rev_pose = *pose;
    AM_xy offset = { robot_radius, 0.0 };
    rev_pose.angle = AM_math_angle_add_f(rev_pose.angle, ir_receiver_distributions(which));
    AM_Pose2D_transform_xy(&rev_pose, &offset, &rev_pose.xy);
    //am_uint8 ir = get_ir_with_beacons(beacons);
    IRPoint ir_pose = { xy_int(rev_pose.xy.x), xy_int(rev_pose.xy.y), theta_int(rev_pose.angle), ir};

    if (IRTraj_size(traj) > 0) {
        am_uint8 tail = traj->tail == 0 ? traj->capacity-1 : traj->tail-1;
        am_uint8 head = traj->head;

        int c = 0;
        while (tail != head && c++ < 10) {
            IRPoint *p = IRTraj_get(traj, tail);
            if (p->ir == ir &&
                IRPoint_distance(p, &ir_pose) < 0.05 && 
                IRPoint_diff_angle(p, &ir_pose) < 10*M_PI_F/180) {
                irmap_mutex_unlock(lock);
                return;
            }
            if (tail == 0)
                tail = traj->capacity;
            --tail;
        }
    }
    printf("  [%d,%d,%d]->%d(w=%d)(s=%d)\n\r", ir_pose.x, ir_pose.y, theta_degree(ir_pose.theta), ir, which, IRTraj_size(traj));
    IRTraj_push_back(traj, &ir_pose);
    _fence_count++;
    irmap_mutex_unlock(lock);

    // debug
    /*int size = IRTraj_size(&_traj);
    if (size > 0 && (size % 8) == 0) {
        AM_Pose2D dock_pose;
        int inlier, total;
        ir_map_guess_dock_pose(&dock_pose, &inlier, &total);
    }*/
}

void ir_map_clear_traj(IRTraj *traj) {
    void *lock = irmap_mutex_lock();
    IRTraj_clear(traj);
    irmap_mutex_unlock(lock);
}

int ir_map_traj_size(IRTraj *traj) {
    void *lock = irmap_mutex_lock();
    int s = IRTraj_size(traj);
    irmap_mutex_unlock(lock);
    return s;
}

am_boolean ir_map_get_last_pose(IRTraj *traj, AM_Pose2D *pose) {
    void *lock = irmap_mutex_lock();
    if (traj->count > 0) {
        am_uint8 tail = traj->tail == 0 ? traj->capacity-1 : traj->tail-1;
        IRPoint *p = IRTraj_get(traj, tail);
        pose->xy.x = xy_float(p->x);
        pose->xy.y = xy_float(p->y);
        pose->angle = theta_float(p->theta);
        irmap_mutex_unlock(lock);
        return TRUE;
    }
    irmap_mutex_unlock(lock);
    return FALSE;
}
    

am_boolean ir_map_get_last_certer_pose(IRTraj *traj, AM_Pose2D *pose) {
    void *lock = irmap_mutex_lock();
    am_uint8 tail = traj->tail == 0 ? traj->capacity-1 : traj->tail-1;
    am_uint8 head = traj->head;
    while (tail != head) {
        IRPoint *p = IRTraj_get(traj, tail);
        if (ir_in_center(p->ir)) {
            pose->xy.x = xy_float(p->x);
            pose->xy.y = xy_float(p->y);
            pose->angle = theta_float(p->theta);
            irmap_mutex_unlock(lock);
            return TRUE;
        }
        if (tail == 0)
            tail = traj->capacity;
        --tail;

    }
    pose->xy.x = 0.;
    pose->xy.y = 0.;
    pose->angle = 0.;
    irmap_mutex_unlock(lock);
    return FALSE;
}

// v3

static am_boolean angle_between(float amin, float amax, float angle) {
    if (amin <= amax)
        return amin <= angle && angle <= amax;
    else
        return (amin <= angle && angle <= M_PI_F) || (-M_PI_F <= angle && angle <= amax);
}

static int map_indicator = 0;
static float get_next_distance_with_ir(int ir) {
    int indicator_bak = map_indicator;
    int dock_w = 10;
    int dock_h = 0;
    while (++map_indicator < 400) {
        if (ir_match(ir_map(map_indicator), ir)) {
            int w = map_indicator % 20;
            int h = map_indicator / 20;
            return 0.1f * sqrt(1.0*(dock_w-w)*(dock_w-w) + (dock_h-h)*(dock_h-h));
        }
    }
    map_indicator = -1;
    while (++map_indicator < indicator_bak) {
        if (ir_match(ir_map(map_indicator), ir)) {
            int w = map_indicator % 20;
            int h = map_indicator / 20;
            return 0.1f * sqrt(1.0*(dock_w-w)*(dock_w-w) + (dock_h-h)*(dock_h-h));
        }
    }
    printf("err fnd %d\n\r", ir);
    return 0.f;
}

typedef void IRMapGenerator(int K, const IRPoint* p[], Hypotheses* hypotheses);
typedef am_boolean IRMapTest(const IRPoint* point, const AM_Pose2D* dock);

am_boolean IRMapTestV3(const IRPoint* point, const AM_Pose2D* dock) {
    AM_Pose2D pose = { xy_float(point->x), xy_float(point->y), theta_float(point->theta) };
    float angle_min = AM_math_angle_add_f(dock->angle, ir_receiver_range_min);
    float angle_max = AM_math_angle_add_f(dock->angle, ir_receiver_range_max);
    am_boolean angle_match = (ir_no_signal(point->ir) || angle_between(angle_min, angle_max, pose.angle));
    return angle_match && ir_match(get_ir_with_pose(&pose, dock), point->ir);
}

static int show_guess_pose = 0; // for debug
void IRMapSolverV3(int K, const IRPoint* p[], Hypotheses* hypotheses) {
    const float angle_interval = M_PI_F / 8;
    const float angle_start = -M_PI_F / 4;
    const size_t angle_count = 5;

    if (Hypotheses_size(hypotheses) == Hypotheses_capacity(hypotheses))
        return;

    AM_Pose2D pose = { xy_float(p[0]->x), xy_float(p[0]->y), theta_float(p[0]->theta) };
    pose.angle = AM_math_angle_add_f(pose.angle, angle_start);

    AM_xy offset = { 0.0, 0.0 };

    for (int i = 0; i < angle_count; ++i) {
        offset.x = get_next_distance_with_ir(p[0]->ir);
        AM_Pose2D dock_pose;
        AM_Pose2D_transform_xy(&pose, &offset, &dock_pose.xy);
        dock_pose.angle = AM_math_angle_add_f(pose.angle, M_PI_F);

        am_boolean valid = TRUE;
        for (int j = 1; j < K; ++j) {
            valid = IRMapTestV3(p[j], &dock_pose);
            if (!valid)
                break;
        }
        if (valid) {
            //if (show_guess_pose == 1)
            //    printf("gues(%d,%d,%d)\n", I(dock_pose.x), I(dock_pose.y), I(dock_pose.theta));
            Hypotheses_push(hypotheses, &dock_pose);
        }
        pose.angle = AM_math_angle_add_f(pose.angle, angle_interval);
    }
}

// functions from ransac.hpp, specialize in c
unsigned int system_rand(unsigned int N) {
    //return rand() % N;
    return (get_random() + get_random()) % N; // for save rand()'s ro/rw
}
void choose_random_subset(size_t N, int K, size_t out[])
{
    for (int i=0; i<K;) {
gen_num:
        size_t r = system_rand(N);
        for (int j=0; j<i; ++j)
            if (out[j] == r)
                goto gen_num;
        out[i++] = r;        
    }
}

void generate_hypotheses(int K, IRTraj* matches, size_t samples,
                         IRMapGenerator generator, Hypotheses* hypotheses)
{
    //assert(K < 6);
    size_t indices[6];
    const IRPoint* sample[6];

    for (size_t i=0; i<samples; ++i) {
        choose_random_subset(IRTraj_size(matches), K, indices);
        for (int j=0; j<K; ++j) {
            sample[j] = IRTraj_get(matches, indices[j]);
        }
        generator(K, sample, hypotheses);
        if (Hypotheses_size(hypotheses) == Hypotheses_capacity(hypotheses))
            return;
    }
}

size_t stdmin(size_t a, size_t b) {
    return a > b ? b : a;
}

size_t preemptive_ransac_weighted(IRTraj* matches, Hypotheses* hypothesis, size_t test_block, IRMapTest test, 
                         AM_Pose2D* result, am_boolean inlier[], float first_weight, float last_weight)
{
    int hypotheses_size = Hypotheses_size(hypothesis);
    int matches_size = IRTraj_size(matches);
    if (hypotheses_size == 0) {
        for (int i = 0; i < IRTraj_capacity(matches); ++i)
            if (inlier != NULL)
                inlier[i] = FALSE;
        return 0;
    }
    //score holds the number
    struct {
        size_t first;
        size_t second;
    } score[IR_HYPOTHESES_EACH];
    for (size_t i=0; i<IR_HYPOTHESES_EACH; ++i) {
    	score[i].first = 0;
    	score[i].second = i;
    }
    
    for (size_t i=0; i!=IRTraj_size(matches) && hypotheses_size > 1; ) {
		size_t next = stdmin(i+test_block, matches_size);
		for (size_t j=0; j<hypotheses_size; ++j) {
			const AM_Pose2D* model = Hypotheses_get(hypothesis, score[j].second);
			/*float count = 0;
			for (size_t k=i; k!=next; ++k)
				count += test(IRTraj_get(matches, k), model) * (first_weight + (last_weight-first_weight) * k / matches_size);
			score[j].first += (size_t)count;*/
			size_t count = 0;
			for (size_t k=i; k!=next; ++k)
				count += test(IRTraj_get(matches, k), model);
			score[j].first += count;

		}
		i = next;
    }
    //size_t argbest = std::max_element(score.begin(), score.end())->second;
    size_t argbest = 0;
    size_t max_score = 0;
    for (int i = 0; i < hypotheses_size; ++i) {
        if (score[i].first > max_score) {
            max_score = score[i].first;
            argbest = i;
        }
    }
    *result = *Hypotheses_get(hypothesis, argbest);

    size_t count = 0;
    for (size_t i=0; i!=matches_size; ++i) {
        am_boolean t = test(IRTraj_get(matches, i), result);
        if (inlier != NULL)
            inlier[i] = t;
    	count += t;
    }

    return count;
}

//static Hypotheses hypotheses;

// different from preemptive_ransac:
// 1. weighted
// 2. inliners omitted
// 3. loop for ransac
size_t ransac_weighted(int K, IRTraj* matches, size_t samples,
    IRMapGenerator generator, size_t test_block, IRMapTest test,
    AM_Pose2D* result, am_boolean inliers[], float first_weight, float last_weight, Hypotheses* hypotheses)
{
    AM_Pose2D best_result;
    size_t best_inliers = 0;
    for (int i = 0; i < IR_HYPOTHESES_TIMES; ++i) {
        Hypotheses_reset(hypotheses);
        generate_hypotheses(K, matches, samples/IR_HYPOTHESES_TIMES, generator, hypotheses);
        //printf("hy%d size=%d\n\r", i, Hypotheses_size(&hypotheses));
        int in = preemptive_ransac_weighted(matches, hypotheses, test_block, test, result, inliers, first_weight, last_weight);
        if (in > best_inliers) {
            best_result = *result;
            best_inliers = in;
        }
    }
    *result = best_result;
    return best_inliers;
}

//static IRTraj traj_copy;
int ir_map_guess_dock_pose(IRTraj *traj, IRTraj *traj_copy, Hypotheses* hypotheses, AM_Pose2D* dock_pose, int* inlier, int* total) {
    //printf("ir_map_guess_dock_pose()\n\r");
    AM_Pose2D ir_dock_pose = {0., 0., 0.};

    int traj_size = ir_map_traj_size(traj);
    if (traj_size < 4) {
        printf("too few ir.\n\r");
        *inlier = 0;
        *total = traj_size;
        return 0;
    }

    size_t inlier_count;
    am_uint32 timer_start = timer_ms();

    void *lock = irmap_mutex_lock();
    IRTraj_copy(traj, traj_copy);
    irmap_mutex_unlock(lock);

    inlier_count = ransac_weighted(4, traj_copy, traj_size * 12, IRMapSolverV3, traj_size, IRMapTestV3, &ir_dock_pose, NULL, 0.1, 1.9, hypotheses);

    am_uint32 timer_end = timer_ms();
    *dock_pose = ir_dock_pose;
    *inlier = inlier_count;
    *total = traj_size;

    int confidence = (int)(100.f*inlier_count/traj_size);
    printf("dock inl=%d/%d(%d),pose=(%d,%d,%d),time=%d\n\r",
        inlier_count, traj_size, confidence,
        I(ir_dock_pose.xy.x), I(ir_dock_pose.xy.y), I(ir_dock_pose.angle),
        timer_end - timer_start);

    return confidence;
}

void update_beacon_for_irmap(u8 ir[]) {
    AM_Pose2D robot_pose;
    localization_get_global_pos(&robot_pose);
    for (int i = 0; i < IR_TIP_SIGNAL_COUNT; ++i) {
        am_uint8 beacon = ir[i];
        am_uint8 ir = get_ir_with_beacons(beacon);
        if (ir != 0) {
            ir_map_add_traj_pose(_traj, &robot_pose, ir, i);
        }
    }
}

// pattern 
#ifdef DOCK_EASY_TEST

///
/// common
s16 pattern_default_max_vel() {
    return 0;
}

///
/// pattern pose
int pattern_pose_init(AM_Pose *pose) {
}

void pattern_pose_get_target(AM_xy *target) {
}

BOOLEAN pattern_pose_gen_point(u32 index, AM_xy *p) {
}


///
/// pattern cycle squarewave
#define INITIAL_ROBOT_HEADING      (0)//(M_PI_F/2.0f)
static struct {
    am_real m_fLength;
    am_real m_fWidth;
    am_real m_fRankWidth;
    int wps_size;
    int round_size;
    am_boolean m_turn_right;
    AM_Pose2D m_start_pose;
} _cs;

int pattern_cycle_squarewave_init(am_real l, am_real w, am_real ww, BOOLEAN turn_right) {
    memset(&_cs, 0, sizeof(_cs));
    _cs.m_fLength = l;
    _cs.m_fWidth = w;
    _cs.m_fRankWidth = ww;
    _cs.m_turn_right = turn_right;
    _cs.round_size = _cs.m_fWidth / _cs.m_fRankWidth * 2 + 1;
    _cs.wps_size = _cs.round_size * 2;
    _cs.wps_size++; // first point is pattern[1]
    localization_get_global_pos(&_cs.m_start_pose);
    _cs.m_start_pose.angle = AM_math_angle_diff_f(_cs.m_start_pose.angle, INITIAL_ROBOT_HEADING);
    return 0;
}

void pattern_cycle_squarewave_get_target(AM_xy *target) {
    target->x = target->y = 100.;
}

BOOLEAN pattern_cycle_squarewave_gen_point(u32 index, AM_xy *p) {
    printf("patn<%d>\r\n", index);
    if (index >= _cs.wps_size)
        return FALSE;

    if (index > _cs.round_size)
        index = 2 * _cs.round_size - index;

    p->x = index / 2 * _cs.m_fRankWidth;
    if (!_cs.m_turn_right)
        p->x = -p->x;


    int t = (index + 1) / 2;
    if (t & 1) {//not even
        p->y = _cs.m_fLength;
    } else {
        p->y = 0;
    }
#if 1
    // eslam: re-coordinate, newx = y, newy = -x
    float x = p->x;
    p->x = p->y;
    p->y = -x;
#endif
    AM_xy pose_in_world;
    robot_to_world(&_cs.m_start_pose, p, &pose_in_world);
    *p = pose_in_world;
    printf(" =>(%d,%d)\r\n", I(p->x), I(p->y));
    return TRUE;
}
s16 pattern_cycle_squarewave_set_max_vel() {
    return 140;
}


///
/// pattern gb
#define WPS_SIZE (15)
#define PATTERN_WPS_SIZE (WPS_SIZE/3)

static struct {
    am_real m_r;
    int wps_size;
    am_int8 m_pattern_series[PATTERN_WPS_SIZE];
    am_real limit_heading;
    am_uint32 turn_speed;    
} _gb;
int pattern_gb_init(am_real r) {
    memset(&_gb, 0, sizeof(_gb));
    _gb.m_r = r;
    _gb.wps_size = WPS_SIZE;
    _gb.m_pattern_series[0] = 0;
    _gb.m_pattern_series[1] = 1;
    _gb.m_pattern_series[2] = -1;
    _gb.m_pattern_series[3] = 2;
    _gb.m_pattern_series[4] = -2;
    _gb.turn_speed = 100;
    _gb.limit_heading = M_PI_F * 2;
    return 0;
}

void pattern_gb_get_target(AM_xy *target) {
    am_real angle = _gb.m_pattern_series[(_gb.wps_size - 1) / 3] * M_PI_F / 4;
    target->x = _gb.m_r  * sin(angle);
    target->y = _gb.m_r  * cos(angle);
}

BOOLEAN pattern_gb_gen_point(u32 index, AM_xy *p) {
    if (index >= _gb.wps_size)
    {
        return FALSE;
    }
    unsigned int nPattern = index % 3;
    am_real angle = 0;
    switch (nPattern)
    {
    case 0: // (0,0)
        turn_circle(_gb.limit_heading, _gb.turn_speed);
        p->x = 0;
        p->y = 0;
        break;
    case 1: // 1m
        angle = _gb.m_pattern_series[index / 3] * M_PI_F / 4;
        p->x = _gb.m_r / 2 * sin(angle);
        p->y = _gb.m_r / 2 * cos(angle);
        break;
    case 2:// 2m
        turn_circle(_gb.limit_heading, _gb.turn_speed);
        angle = _gb.m_pattern_series[index / 3] * M_PI_F / 4;
        p->x = _gb.m_r  * sin(angle);
        p->y = _gb.m_r  * cos(angle);
    default:
        break;
    }
    
    // eslam: re-coordinate, newx = y, newy = -x
    float x = p->x;
    p->x = p->y;
    p->y = -x;
    return TRUE;

}

///
/// pattern squarewave
static struct {
	am_real m_fLength;
	am_real m_fWidth;
	am_real m_fRankWidth;
	int wps_size;
	AM_Pose2D m_start_pose;
} _sq;

int pattern_squarewave_init(am_real l, am_real w, am_real ww) {
    memset(&_sq, 0, sizeof(_sq));
    _sq.m_fLength = l;
    _sq.m_fWidth = w;
    _sq.m_fRankWidth = ww;
    _sq.wps_size = _sq.m_fWidth / _sq.m_fRankWidth * 2;
    localization_get_global_pos(&_sq.m_start_pose);
    return 0;
}

BOOLEAN pattern_squarewave_gen_point(u32 index, AM_xy *p) {
    if (index >= _sq.wps_size)
    {
        return FALSE;
    }
    p->x = index / 2 * _sq.m_fRankWidth;
    int t = (index + 1) / 2;
    if (t & 1)
    {//not even
        p->y = _sq.m_fLength;
    }
    else
    {
        p->y = 0;
    }
    // eslam: re-coordinate, newx = y, newy = -x
    float x = p->x;
    p->x = p->y;
    p->y = -x;
    
    AM_xy pose_in_world;
    robot_to_world(&_sq.m_start_pose, p, &pose_in_world);
    *p = pose_in_world;
    return TRUE;
}

void pattern_squarewave_get_target(AM_xy *target) {
    pattern_squarewave_gen_point(_sq.wps_size - 1, target);
}

s16 pattern_squarewave_set_max_vel() {
    return 100;
}

#else

s16 pattern_default_max_vel() {return 0;}

int pattern_pose_init(AM_Pose *pose) {return 0;}
void pattern_pose_get_target(AM_xy *target) {}
BOOLEAN pattern_pose_gen_point(u32 index, AM_xy *p) {return FALSE;}

int pattern_cycle_squarewave_init(am_real l, am_real w, am_real ww, BOOLEAN turn_right) {return 0;}
void pattern_cycle_squarewave_get_target(AM_xy *target) {}
BOOLEAN pattern_cycle_squarewave_gen_point(u32 index, AM_xy *p) {return FALSE;}
s16 pattern_cycle_squarewave_set_max_vel(){return 0;};

int pattern_gb_init(am_real r) {return 0;}
void pattern_gb_get_target(AM_xy *target) {}
BOOLEAN pattern_gb_gen_point(u32 index, AM_xy *p) {return FALSE;}

int pattern_squarewave_init(am_real l, am_real w, am_real ww) {return 0;}
BOOLEAN pattern_squarewave_gen_point(u32 index, AM_xy *p) {return FALSE;}
void pattern_squarewave_get_target(AM_xy *target){}
s16 pattern_squarewave_set_max_vel(){return 0;}

#endif

///
/// pattern circle [and half circle]

#define POLYGON_COUNT (12)
#define MAX_GENERATION_POINTS (120)
#define MAX_VELOCITY_TYPE0 (240)
#define MAX_VELOCITY_TYPE1 (180)

static struct {
    am_real m_r;
    AM_xy m_points[POLYGON_COUNT];
    u8 m_descs[POLYGON_COUNT];
    u8 m_type;
    u8 m_index;
    u8 m_max;
    u8 m_gen_count; // caution: overflow
    BOOLEAN m_clockwise;
    BOOLEAN should_wall_follow;
} _circle;

static BOOLEAN circle_GenerateCirclePattern() {
    if (!_circle.should_wall_follow && _circle.m_gen_count > MAX_GENERATION_POINTS)
        return FALSE;
    if (_circle.should_wall_follow && _circle.m_gen_count > 0)
        return FALSE;
    AM_Pose current, target;
    robot_pos_get(&current);
    AM_xy offset = { _circle.m_r, 0.0 };
    am_real angle = M_PI_F * 2 / POLYGON_COUNT;
    if (_circle.m_clockwise) angle = -angle;
    _circle.m_max = POLYGON_COUNT;
    for (int i = 0; i < _circle.m_max; ++i) {
        AM_Pose2D_transform_xy(&current, &offset, &target.xy);
        _circle.m_points[i] = target.xy;
        _circle.m_descs[i] = i;
        current.angle = AM_math_angle_add_f(current.angle, angle);
        printf("wp[%d]\r\n", i);
        printf(" =>(%d,%d)\r\n", I(_circle.m_points[i].x), I(_circle.m_points[i].y));
    }
    _circle.m_clockwise = !_circle.m_clockwise;
    _circle.m_index = 0;
    _circle.m_gen_count += _circle.m_max;
    return TRUE;
}
static BOOLEAN circle_GenerateHalfCirclePattern() {
    if (_circle.m_gen_count > 0) /* generate only once */
        return FALSE;
    AM_Pose current, target;
    robot_pos_get(&current);
    AM_xy offset = { -_circle.m_r, 0.0 };
    AM_Pose2D_transform_xy(&current, &offset, &target.xy);
    current.xy = target.xy; // circle center

    offset.x = _circle.m_r;
    am_real angle = M_PI_F * 2 / POLYGON_COUNT;
    if (_circle.m_clockwise) angle = -angle;
    _circle.m_max = POLYGON_COUNT / 2 + 1 + 2;
    am_real start_angle = -(angle * (int)(_circle.m_max / 2));
    current.angle = AM_math_angle_add_f(current.angle, start_angle);
    for (int i = 0; i < _circle.m_max; ++i) {
        AM_Pose2D_transform_xy(&current, &offset, &target.xy);
        _circle.m_points[i] = target.xy;
        _circle.m_descs[i] = i;
        current.angle = AM_math_angle_add_f(current.angle, angle);
        printf("wp[%d]\r\n", i);
        printf(" =>(%d,%d)\r\n", I(_circle.m_points[i].x), I(_circle.m_points[i].y));
    }
    _circle.m_clockwise = !_circle.m_clockwise;
    _circle.m_index = 0;
    _circle.m_gen_count += _circle.m_max;
    return TRUE;
}
static BOOLEAN circle_GeneratePattern() {
    if (_circle.m_type == 0)
        return circle_GenerateCirclePattern();
    else if (_circle.m_type == 1)
        return circle_GenerateHalfCirclePattern();
    return FALSE;
}

static BOOLEAN circle_IsInvalidPoint(AM_Pose* bump_pose, AM_xy* point) {
    am_real dx = point->x - bump_pose->xy.x;
    am_real dy = point->y - bump_pose->xy.y;
    am_real angle = atan2f(dy, dx);
    am_real diff = AM_math_angle_diff_f(angle, bump_pose->angle);
    return -M_PI_F / 3 < diff && diff < M_PI_F / 3;
}


void pattern_circle_get_target(AM_xy *target) {
    *target = _circle.m_points[_circle.m_max - 1];
}

BOOLEAN pattern_circle_gen_point(u32 index, AM_xy *p) {
    printf("get pt %d/%d\r\n", _circle.m_index, _circle.m_max);
    if (_circle.m_index >= _circle.m_max) {
        if (!circle_GeneratePattern()) {
            return FALSE;
        }
    }
    *p = _circle.m_points[_circle.m_index];
    printf(" =>%d\r\n", _circle.m_descs[_circle.m_index]);
    _circle.m_index++;
    return TRUE;
}

int pattern_circle_init(int type, am_real r, BOOLEAN clockwise, BOOLEAN wf) {
    memset(&_circle, 0, sizeof(_circle));
    _circle.m_type = type;
    _circle.m_r = r;
    _circle.m_clockwise = clockwise;
    _circle.should_wall_follow = wf;
    _circle.m_gen_count = 0;
    circle_GeneratePattern();
    return 0;
}

s16 pattern_circle_set_max_vel() {
    if (_circle.m_type == 0)
        return MAX_VELOCITY_TYPE0;
    else if (_circle.m_type == 1)
        return MAX_VELOCITY_TYPE1;
    return 0;
}

void pattern_circle_strip(AM_Pose* strip_pose) {
    int index = _circle.m_index;
    while (index < _circle.m_max) {
        if (circle_IsInvalidPoint(strip_pose, &_circle.m_points[index])) {
            printf("strip %d\r\n", _circle.m_descs[index]);
            for (int i = index; i < _circle.m_max - 1; ++i) {
                _circle.m_points[i] = _circle.m_points[i + 1];
                _circle.m_descs[i] = _circle.m_descs[i + 1];
            }
            _circle.m_max -= 1;
        }
        else {
            index++;
        }
    }
    printf("strip c=%d,%d\r\n", _circle.m_index, _circle.m_max);
}

