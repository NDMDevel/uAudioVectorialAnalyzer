/* 
 * File:   uAVAlib.h
 * Author: Damian
 *
 * Created on 12 de octubre de 2014, 00:02
 */

#ifndef UAVALIB_H
#define	UAVALIB_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <xc.h>
#include <stdint.h>

#define FCY 40000000
#include <libpic30.h>

void SetDev40MIPS(void);


#ifdef	__cplusplus
}
#endif

#endif	/* UAVALIB_H */

