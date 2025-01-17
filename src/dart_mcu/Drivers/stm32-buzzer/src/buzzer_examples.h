#ifndef BUZZER_EXAMPLES_H
#define BUZZER_EXAMPLES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "buzzer_tones.h"

enum song_list {
    Eautopilot_disconnect,
    Ereconnect,
    Ewinxp,
    Elaoda,
    Estartup,
    Eplug_in,
    Eremove,
    Eprotect
};

// Disable Autopilot Beep
static note_t buzzer_autopilot_disconnect[] = {
        {NOTE_GS6, 8 * 7},
        {NOTE_GS5, 7 * 7},
        {0,        4 * 7},
        {NOTE_GS6, 15 * 7},
        {0,        15 * 7},
        {NOTE_GS6, 8 * 7},
        {NOTE_GS5, 7 * 7},
        {0,        4 * 7},
        {NOTE_GS6, 15 * 7},
        {0,        15 * 7},
        {NOTE_GS6, 8 * 7},
        {NOTE_GS5, 7 * 7},
        {0,        4 * 7},
        {NOTE_GS6, 15 * 7},
        {0,        45 * 7}
};

static note_t buzzer_dji_startup[] = {
        {NOTE_C5, 250},
        {NOTE_D5, 250},
        {NOTE_G5, 450},
        {0,       200}
};

static note_t buzzer_winxp[] = { // DS5 DS4 AS4 GS4 DS5 AS4
        {NOTE_DS6, 300},
        {NOTE_DS5, 150},
        {NOTE_AS5, 250},
        {NOTE_GS5, 500},
        {NOTE_DS6, 300},
        {NOTE_AS5, 800},
        {0,        200}
};

// BGM_LAODA
static note_t buzzer_laoda[] = {
        {698,  355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {932,  355},
        {0,    23},
        {587,  729},
        {0,    23},
        {932,  167},
        {0,    23},
        {1047, 167},
        {0,    23},
        {1175, 167},
        {0,    23},
        {1047, 167},
        {0,    23},
        {932,  167},
        {0,    23},
        {1047, 167},
        {0,    23},
        {698,  355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {932,  355},
        {0,    23},
        {587,  729},
        {0,    23},
        {932,  167},
        {0,    23},
        {1047, 167},
        {0,    23},
        {1175, 167},
        {0,    23},
        {1047, 167},
        {0,    23},
        {932,  167},
        {0,    23},
        {1047, 167},
        {0,    23},
        {698,  355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {932,  355},
        {0,    23},
        {698,  729},
        {0,    23},
        {932,  167},
        {0,    23},
        {1047, 167},
        {0,    23},
        {1175, 167},
        {0,    23},
        {1047, 167},
        {0,    23},
        {932,  167},
        {0,    23},
        {1047, 167},
        {0,    23},
        {698,  355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {932,  355},
        {0,    23},
        {698,  729},
        {0,    23},
        {466,  355},
        {0,    23},
        {587,  355},
        {0,    23},
        {698,  355},
        {0,    23},
        {784,  1105},
        {0,    23},
        {698,  1667},
        {0,    23},
        {466,  167},
        {0,    23},
        {523,  355},
        {0,    23},
        {523,  355},
        {0,    23},
        {466,  355},
        {0,    23},
        {587,  1479},
        {0,    23},
        {587,  167},
        {0,    23},
        {698,  167},
        {0,    23},
        {784,  355},
        {0,    23},
        {880,  355},
        {0,    23},
        {784,  355},
        {0,    23},
        {698,  355},
        {0,    23},
        {587,  355},
        {0,    23},
        {523,  355},
        {0,    23},
        {523,  355},
        {0,    23},
        {466,  355},
        {0,    23},
        {523,  355},
        {0,    23},
        {523,  355},
        {0,    23},
        {587,  355},
        {0,    23},
        {466,  1125},
        {0,    100},
        {0,    94},
        {523,  167},
        {0,    23},
        {587,  167},
        {0,    23},
        {698,  167},
        {0,    23},
        {784,  1105},
        {0,    23},
        {698,  1667},
        {0,    23},
        {466,  167},
        {0,    23},
        {523,  355},
        {0,    23},
        {523,  355},
        {0,    23},
        {466,  355},
        {0,    23},
        {587,  1105},
        {0,    23},
        {587,  355},
        {0,    23},
        {698,  355},
        {0,    23},
        {784,  355},
        {0,    23},
        {932,  355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {1175, 355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {932,  355},
        {0,    23},
        {784,  355},
        {0,    23},
        {932,  355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {932,  355},
        {0,    23},
        {932,  1105},
        {0,    23},
        {784,  355},
        {0,    23},
        {932,  355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {932,  355},
        {0,    23},
        {932,  1875},
        {0,    100},
        {370,  542},
        {0,    23},
        {330,  167},
        {0,    23},
        {370,  542},
        {0,    23},
        {330,  167},
        {0,    23},
        {370,  542},
        {0,    23},
        {330,  167},
        {0,    23},
        {370,  167},
        {0,    23},
        {440,  167},
        {0,    23},
        {370,  167},
        {0,    23},
        {330,  167},
        {0,    23},
        {294,  542},
        {0,    23},
        {247,  167},
        {0,    23},
        {294,  355},
        {0,    23},
        {330,  167},
        {0,    23},
        {370,  560},
        {0,    250},
        {0,    321},
        {0,    188},
        {294,  167},
        {0,    23},
        {247,  167},
        {0,    23},
        {294,  542},
        {0,    23},
        {247,  167},
        {0,    23},
        {294,  355},
        {0,    23},
        {330,  167},
        {0,    23},
        {370,  560},
        {0,    250},
        {0,    321},
        {0,    188},
        {294,  167},
        {0,    23},
        {247,  167},
        {0,    23},
        {294,  542},
        {0,    23},
        {247,  167},
        {0,    23},
        {294,  355},
        {0,    23},
        {330,  167},
        {0,    23},
        {294,  560},
        {0,    250},
        {0,    321},
        {0,    188},
        {294,  167},
        {0,    23},
        {330,  167},
        {0,    23},
        {370,  542},
        {0,    23},
        {330,  167},
        {0,    23},
        {370,  542},
        {0,    23},
        {330,  167},
        {0,    23},
        {370,  542},
        {0,    23},
        {330,  167},
        {0,    23},
        {370,  167},
        {0,    23},
        {440,  167},
        {0,    23},
        {370,  167},
        {0,    23},
        {330,  167},
        {0,    23},
        {294,  542},
        {0,    23},
        {247,  167},
        {0,    23},
        {294,  355},
        {0,    23},
        {330,  167},
        {0,    23},
        {370,  560},
        {0,    250},
        {0,    321},
        {0,    188},
        {294,  167},
        {0,    23},
        {247,  167},
        {0,    23},
        {294,  542},
        {0,    23},
        {247,  167},
        {0,    23},
        {294,  355},
        {0,    23},
        {330,  167},
        {0,    23},
        {370,  560},
        {0,    250},
        {0,    321},
        {0,    188},
        {294,  167},
        {0,    23},
        {247,  167},
        {0,    23},
        {294,  542},
        {0,    23},
        {247,  167},
        {0,    23},
        {294,  355},
        {0,    23},
        {330,  167},
        {0,    23},
        {294,  560},
        {0,    750},
        {0,    188},
        {0,    100},
        {392,  1105},
        {0,    23},
        {466,  1105},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {587,  355},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {392,  542},
        {0,    23},
        {587,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {392,  542},
        {0,    23},
        {392,  167},
        {0,    23},
        {587,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  355},
        {0,    23},
        {466,  355},
        {0,    23},
        {587,  355},
        {0,    23},
        {698,  355},
        {0,    23},
        {784,  1105},
        {0,    23},
        {698,  1667},
        {0,    23},
        {466,  167},
        {0,    23},
        {523,  355},
        {0,    23},
        {523,  355},
        {0,    23},
        {466,  355},
        {0,    23},
        {587,  1479},
        {0,    23},
        {587,  167},
        {0,    23},
        {698,  167},
        {0,    23},
        {784,  355},
        {0,    23},
        {880,  355},
        {0,    23},
        {784,  355},
        {0,    23},
        {698,  355},
        {0,    23},
        {587,  355},
        {0,    23},
        {523,  355},
        {0,    23},
        {523,  355},
        {0,    23},
        {466,  355},
        {0,    23},
        {523,  355},
        {0,    23},
        {523,  355},
        {0,    23},
        {587,  355},
        {0,    23},
        {466,  1125},
        {0,    100},
        {0,    94},
        {523,  167},
        {0,    23},
        {587,  167},
        {0,    23},
        {698,  167},
        {0,    23},
        {784,  1105},
        {0,    23},
        {698,  1667},
        {0,    23},
        {466,  167},
        {0,    23},
        {523,  355},
        {0,    23},
        {523,  355},
        {0,    23},
        {466,  355},
        {0,    23},
        {587,  1105},
        {0,    23},
        {587,  355},
        {0,    23},
        {698,  355},
        {0,    23},
        {784,  355},
        {0,    23},
        {932,  355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {1175, 355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {932,  355},
        {0,    23},
        {784,  355},
        {0,    23},
        {932,  355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {932,  355},
        {0,    23},
        {932,  1105},
        {0,    23},
        {784,  355},
        {0,    23},
        {932,  355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {932,  355},
        {0,    23},
        {932,  1105},
        {0,    23},
        {932,  355},
        {0,    23},
        {880,  355},
        {0,    23},
        {784,  1105},
        {0,    23},
        {698,  1105},
        {0,    23},
        {932,  355},
        {0,    23},
        {880,  355},
        {0,    23},
        {784,  542},
        {0,    23},
        {880,  167},
        {0,    23},
        {784,  355},
        {0,    23},
        {698,  355},
        {0,    23},
        {587,  729},
        {0,    23},
        {349,  167},
        {0,    23},
        {392,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {523,  167},
        {0,    23},
        {587,  542},
        {0,    23},
        {523,  167},
        {0,    23},
        {587,  542},
        {0,    23},
        {523,  167},
        {0,    23},
        {587,  542},
        {0,    23},
        {523,  167},
        {0,    23},
        {587,  167},
        {0,    23},
        {698,  167},
        {0,    23},
        {587,  167},
        {0,    23},
        {523,  167},
        {0,    23},
        {466,  542},
        {0,    23},
        {392,  167},
        {0,    23},
        {466,  355},
        {0,    23},
        {523,  167},
        {0,    23},
        {466,  1310},
        {0,    250},
        {0,    321},
        {370,  542},
        {0,    23},
        {330,  167},
        {0,    23},
        {370,  542},
        {0,    23},
        {330,  167},
        {0,    23},
        {370,  542},
        {0,    23},
        {330,  167},
        {0,    23},
        {370,  167},
        {0,    23},
        {440,  167},
        {0,    23},
        {370,  167},
        {0,    23},
        {330,  167},
        {0,    23},
        {294,  542},
        {0,    23},
        {247,  167},
        {0,    23},
        {294,  355},
        {0,    23},
        {330,  167},
        {0,    23},
        {370,  560},
        {0,    250},
        {0,    321},
        {0,    188},
        {294,  167},
        {0,    23},
        {247,  167},
        {0,    23},
        {294,  542},
        {0,    23},
        {247,  167},
        {0,    23},
        {294,  355},
        {0,    23},
        {330,  167},
        {0,    23},
        {370,  560},
        {0,    250},
        {0,    321},
        {0,    188},
        {294,  167},
        {0,    23},
        {247,  167},
        {0,    23},
        {294,  542},
        {0,    23},
        {247,  167},
        {0,    23},
        {294,  355},
        {0,    23},
        {330,  167},
        {0,    23},
        {294,  938},
        {0,    250},
        {0,    321},
        {0,    188},
        {370,  542},
        {0,    23},
        {330,  167},
        {0,    23},
        {370,  542},
        {0,    23},
        {330,  167},
        {0,    23},
        {370,  542},
        {0,    23},
        {330,  167},
        {0,    23},
        {370,  167},
        {0,    23},
        {440,  167},
        {0,    23},
        {370,  167},
        {0,    23},
        {330,  167},
        {0,    23},
        {294,  542},
        {0,    23},
        {247,  167},
        {0,    23},
        {294,  355},
        {0,    23},
        {330,  167},
        {0,    23},
        {370,  560},
        {0,    250},
        {0,    321},
        {0,    188},
        {294,  167},
        {0,    23},
        {247,  167},
        {0,    23},
        {294,  542},
        {0,    23},
        {247,  167},
        {0,    23},
        {294,  355},
        {0,    23},
        {330,  167},
        {0,    23},
        {370,  560},
        {0,    250},
        {0,    321},
        {0,    188},
        {294,  167},
        {0,    23},
        {247,  167},
        {0,    23},
        {294,  542},
        {0,    23},
        {247,  167},
        {0,    23},
        {294,  355},
        {0,    23},
        {330,  167},
        {0,    23},
        {294,  560},
        {0,    750},
        {0,    100},
        {0,    94},
        {294,  1105},
        {0,    23},
        {349,  1105},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {587,  355},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {392,  542},
        {0,    23},
        {587,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {392,  542},
        {0,    23},
        {392,  167},
        {0,    23},
        {587,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {466,  355},
        {0,    23},
        {466,  355},
        {0,    23},
        {587,  355},
        {0,    23},
        {698,  355},
        {0,    23},
        {784,  1105},
        {0,    23},
        {698,  1105},
        {0,    23},
        {466,  355},
        {0,    23},
        {523,  729},
        {0,    23},
        {587,  1125},
        {0,    300},
        {0,    83},
        {466,  375},
        {0,    750},
        {0,    300},
        {0,    83},
        {784,  355},
        {0,    23},
        {784,  355},
        {0,    23},
        {784,  167},
        {0,    23},
        {784,  542},
        {0,    23},
        {698,  355},
        {0,    23},
        {698,  355},
        {0,    23},
        {587,  167},
        {0,    23},
        {698,  542},
        {0,    23},
        {784,  1125},
        {0,    750},
        {0,    100},
        {0,    94},
        {523,  167},
        {0,    23},
        {523,  167},
        {0,    23},
        {587,  167},
        {0,    23},
        {622,  355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {932,  2229},
        {0,    23},
        {1175, 729},
        {0,    23},
        {1047, 729},
        {0,    23},
        {932,  729},
        {0,    23},
        {880,  729},
        {0,    23},
        {784,  1479},
        {0,    23},
        {880,  1105},
        {0,    23},
        {932,  167},
        {0,    23},
        {1047, 355},
        {0,    23},
        {784,  1310},
        {0,    300},
        {0,    83},
        {932,  355},
        {0,    23},
        {1175, 355},
        {0,    23},
        {1397, 355},
        {0,    23},
        {1568, 1105},
        {0,    23},
        {1397, 1667},
        {0,    23},
        {932,  167},
        {0,    23},
        {1047, 355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {932,  355},
        {0,    23},
        {1175, 1479},
        {0,    23},
        {1175, 167},
        {0,    23},
        {1397, 167},
        {0,    23},
        {1568, 355},
        {0,    23},
        {1760, 355},
        {0,    23},
        {1568, 355},
        {0,    23},
        {1397, 355},
        {0,    23},
        {1175, 355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {932,  355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {1175, 355},
        {0,    23},
        {932,  1125},
        {0,    100},
        {0,    94},
        {1047, 167},
        {0,    23},
        {1175, 167},
        {0,    23},
        {1397, 167},
        {0,    23},
        {1568, 1105},
        {0,    23},
        {1397, 1667},
        {0,    23},
        {932,  167},
        {0,    23},
        {1047, 355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {932,  355},
        {0,    23},
        {1175, 1105},
        {0,    23},
        {1175, 355},
        {0,    23},
        {1397, 355},
        {0,    23},
        {1568, 355},
        {0,    23},
        {1865, 375},
        {0,    750},
        {0,    300},
        {0,    83},
        {1865, 355},
        {0,    23},
        {1568, 355},
        {0,    23},
        {1865, 375},
        {0,    250},
        {0,    321},
        {0,    188},
        {1865, 355},
        {0,    23},
        {1865, 1105},
        {0,    23},
        {784,  355},
        {0,    23},
        {932,  355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {1047, 355},
        {0,    23},
        {932,  355},
        {0,    23},
        {932,  1105},
        {0,    23},
        {932,  355},
        {0,    23},
        {880,  355},
        {0,    23},
        {784,  1105},
        {0,    23},
        {698,  1105},
        {0,    23},
        {932,  355},
        {0,    23},
        {880,  355},
        {0,    23},
        {784,  542},
        {0,    23},
        {880,  167},
        {0,    23},
        {784,  355},
        {0,    23},
        {698,  355},
        {0,    23},
        {587,  729},
        {0,    23},
        {349,  167},
        {0,    23},
        {392,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {523,  167},
        {0,    23},
        {587,  542},
        {0,    23},
        {523,  167},
        {0,    23},
        {587,  542},
        {0,    23},
        {523,  167},
        {0,    23},
        {587,  542},
        {0,    23},
        {523,  167},
        {0,    23},
        {587,  167},
        {0,    23},
        {698,  167},
        {0,    23},
        {587,  167},
        {0,    23},
        {523,  167},
        {0,    23},
        {466,  542},
        {0,    23},
        {392,  167},
        {0,    23},
        {466,  355},
        {0,    23},
        {523,  167},
        {0,    23},
        {466,  917},
        {0,    23},
        {932,  355},
        {0,    23},
        {880,  355},
        {0,    23},
        {784,  1105},
        {0,    23},
        {698,  1105},
        {0,    23},
        {932,  355},
        {0,    23},
        {880,  355},
        {0,    23},
        {784,  542},
        {0,    23},
        {880,  167},
        {0,    23},
        {784,  355},
        {0,    23},
        {698,  355},
        {0,    23},
        {587,  729},
        {0,    23},
        {349,  167},
        {0,    23},
        {392,  167},
        {0,    23},
        {466,  167},
        {0,    23},
        {523,  167},
        {0,    23},
        {587,  542},
        {0,    23},
        {523,  167},
        {0,    23},
        {587,  542},
        {0,    23},
        {523,  167},
        {0,    23},
        {587,  542},
        {0,    23},
        {523,  167},
        {0,    23},
        {587,  167},
        {0,    23},
        {698,  167},
        {0,    23},
        {587,  167},
        {0,    23},
        {523,  167},
        {0,    23},
        {466,  542},
        {0,    23},
        {392,  167},
        {0,    23},
        {466,  355},
        {0,    23},
        {523,  167},
        {0,    23},
        {466,  3188}
};

static note_t buzzer_startup[] = {
        {NOTE_G5, 8 * 15},    // 第一个音符
        {NOTE_E6, 8 * 15},    // 第二个音符
        {NOTE_C6, 8 * 15},    // 第三个音符
        {NOTE_G6, 8 * 15},    // 第四个音符
        {NOTE_E7, 8 * 15},    // 第五个音符
};

static note_t buzzer_plug_in[] = {
        {NOTE_G5, 100},
        {NOTE_D6, 100},
        {0,       100}
};


static note_t buzzer_remove[] = {
        {NOTE_D6, 100},
        {NOTE_G5, 100},
        {0,       100}
};

static note_t buzzer_error[] = {
        {NOTE_G5, 180},
        {NOTE_D6, 170},
        {NOTE_C6, 200},
        {0,       100}
};

static note_t buzzer_protect[] = {
        {NOTE_GS6, 250},
        {NOTE_DS6, 250},
        {NOTE_GS5, 250},
        {NOTE_AS5, 750},
        {0,       100}
};

#ifdef __cplusplus
};
#endif

#endif