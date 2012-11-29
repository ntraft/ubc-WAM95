#include "stdheader.h"

char* ctrlMode = NULL;
bool vcMode = false;


// Functions that help display data from the Hand's (optional) tactile sensors.
// Note that the palm tactile sensor has a unique cell layout that these
// functions do not print correctly.
const int TACT_CELL_HEIGHT = 3;
const int TACT_CELL_WIDTH = 6;
const int TACT_BOARD_ROWS = 8;
const int TACT_BOARD_COLS = 3;
const int TACT_BOARD_STRIDE = TACT_BOARD_COLS * TACT_CELL_WIDTH + 2;
void drawBoard(WINDOW *win, int starty, int startx, int rows, int cols,
		int tileHeight, int tileWidth);
void graphPressures(WINDOW *win, int starty, int startx,
		const TactilePuck::v_type& pressures);

void drawBoard(WINDOW *win, int starty, int startx, int rows, int cols,
		int tileHeight, int tileWidth) {
	int endy, endx, i, j;

	endy = starty + rows * tileHeight;
	endx = startx + cols * tileWidth;

	for (j = starty; j <= endy; j += tileHeight)
		for (i = startx; i <= endx; ++i)
			mvwaddch(win, j, i, ACS_HLINE);
	for (i = startx; i <= endx; i += tileWidth)
		for (j = starty; j <= endy; ++j)
			mvwaddch(win, j, i, ACS_VLINE);
	mvwaddch(win, starty, startx, ACS_ULCORNER);
	mvwaddch(win, endy, startx, ACS_LLCORNER);
	mvwaddch(win, starty, endx, ACS_URCORNER);
	mvwaddch(win, endy, endx, ACS_LRCORNER);
	for (j = starty + tileHeight; j <= endy - tileHeight; j += tileHeight) {
		mvwaddch(win, j, startx, ACS_LTEE);
		mvwaddch(win, j, endx, ACS_RTEE);
		for (i = startx + tileWidth; i <= endx - tileWidth; i += tileWidth)
			mvwaddch(win, j, i, ACS_PLUS);
	}
	for (i = startx + tileWidth; i <= endx - tileWidth; i += tileWidth) {
		mvwaddch(win, starty, i, ACS_TTEE);
		mvwaddch(win, endy, i, ACS_BTEE);
	}
}

void graphCell(WINDOW *win, int starty, int startx, double pressure) {
	int i, chunk;
	char c;

	int value = (int)(pressure * 256.0) / 102;  // integer division
//	int value = (int)(pressure * 256.0) / 50; // integer division
	for (i = 4; i >= 0; --i) {
		chunk = (value <= 7) ? value : 7;
		value -= chunk;

		switch (chunk) {
		default:  c = '#'; break;
		case 2:   c = '~'; break;
		case 1:   c = '-'; break;
		case 0:   c = '_'; break;
		}
		mvwprintw(win, starty + 1, startx + i, "%c", c);

		switch (chunk - 4) {
		case 3:   c = '#'; break;
		case 2:   c = '~'; break;
		case 1:   c = '-'; break;
		case 0:   c = '_'; break;
		default:  c = ' '; break;
		}
		mvwprintw(win, starty, startx + i, "%c", c);
	}
}

void graphPressures(WINDOW *win, int starty, int startx,
		const TactilePuck::v_type& pressures) {
	for (int i = 0; i < pressures.size(); ++i) {
		graphCell(win,
				starty + 1 + TACT_CELL_HEIGHT *
						(TACT_BOARD_ROWS - 1 - (i / 3 /* integer division */)),
				startx + 1 + TACT_CELL_WIDTH * (i % TACT_BOARD_COLS),
				pressures[i]);
	}
}

bool validate_args(int argc, char** argv) {
	switch (argc) {
	case 2:
		if (boost::filesystem::exists(argv[1])) {
			printf("\nTrajectory to be played in current control mode: %s\n\n",
					argv[1]);
			return true;
			break;
		} else {
			printf("\nTrajectory not found in location specified: %s\n\n",
					argv[1]);
			return false;
			break;
		}
	case 3:
		ctrlMode = argv[2];
		if (boost::filesystem::exists(argv[1])
				&& (strcmp(ctrlMode, "cc") == 0 || strcmp(ctrlMode, "-cc") == 0
						|| strcmp(ctrlMode, "vc") == 0
						|| strcmp(ctrlMode, "-vc") == 0)) {
			printf(
					"\nTrajectory to be played in %s mode: %s\n\n",
					strcmp(ctrlMode, "vc") == 0
							|| strcmp(ctrlMode, "-vc") == 0 ?
							"voltage control" : "current control", argv[1]);
			if (strcmp(ctrlMode, "vc") == 0 || strcmp(ctrlMode, "-vc") == 0)
				vcMode = true;
			return true;
			break;
		} else {
			printf("\nTrajectory not found in location specified: %s\n\n",
					argv[1]);
			return false;
			break;
		}
	default:
		printf("Usage: %s <path/to/trajectory> [<Control Mode (cc or vc)>]\n",
				argv[0]);
		return false;
		break;
	}
}
