#ifndef _DISPLAY_H
#define _DISPLAY_H

#include "stdheader.h"

class Display{

    void drawBoard(WINDOW *win, int starty, int startx, int rows, int cols, int tileHeight, int tileWidth); 
    void graphCell(WINDOW *win, int starty, int startx, double pressure); 
    void graphPressures(WINDOW *win, int starty, int startx, const TactilePuck::v_type& pressures); 

};
#endif
