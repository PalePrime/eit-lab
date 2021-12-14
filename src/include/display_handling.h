#ifndef DISPLAY_H
#define DISPLAY_H

#ifdef __cplusplus
extern "C" {
#endif

void createDisplayHandler();

#define SHAPE_CNT 50

extern void display_init();
extern void display_loop();



#ifdef __cplusplus
}
#endif

#endif /*DISPLAY_H*/
