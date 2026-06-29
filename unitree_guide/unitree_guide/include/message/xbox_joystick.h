#define XBOX_BUTTON_A 0
#define XBOX_BUTTON_B 1
#define XBOX_BUTTON_X 2
#define XBOX_BUTTON_Y 3
#define XBOX_BUTTON_LB 4
#define XBOX_BUTTON_RB 5
#define XBOX_BUTTON_SELECT 6
#define XBOX_BUTTON_START 7
#define XBOX_BUTTON_HOME 8
#define XBOX_BUTTON_LO 9
#define XBOX_BUTTON_RO 10

#define XBOX_AXIS_LX 0
#define XBOX_AXIS_LY 1
#define XBOX_AXIS_RX 2
#define XBOX_AXIS_RY 3
#define XBOX_AXIS_LT 5
#define XBOX_AXIS_RT 4
#define XBOX_AXIS_XX 6
#define XBOX_AXIS_YY 7

typedef struct {
    int time;
    int a, b, x, y;
    int lb, rb;
    int select, start, home;
    int lo, ro;
    int lx, ly, rx, ry;
    int lt, rt;
    int xx, yy;
} xbox_map_t;