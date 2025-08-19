/**

Manjiang Cao   <manjiang19@hkust-gz.edu.cn> Atomic. 

*/
#ifndef __MUSIC_H__
#define __MUSIC_H__

typedef struct {
    float real;
    float imag;
} ComplexFloat;


ComplexFloat complex_add(ComplexFloat a, ComplexFloat b);