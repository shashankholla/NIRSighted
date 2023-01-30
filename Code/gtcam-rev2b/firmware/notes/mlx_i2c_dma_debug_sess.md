

{{sta = 0x1, systick = 0x12ac5, label = 0x10}
 {sta = 0x21, systick = 0x12b4a, label = 0x0}
 {sta = 0x1, systick = 0x12b58, label = 0x11}
call to i2cwrite,         STA: TX empty
    interrupt             STA: STOP, TX empty
return into i2cwrite,     STA: TX empty


 {sta = 0x1, systick = 0x12b5c, label = 0x100},
 {sta = 0x8041, systick = 0x12bab, label = 0x0},
 {sta = 0x8041, systick = 0x12bb9, label = 0x101},
 {sta = 0x8021, systick = 0x12bc2, label = 0x0},
 {sta = 0x21, systick = 0x12eed, label = 0x0},
 {sta = 0x1, systick = 0x12ef9, label = 0x102},
call to i2cread                  STA: TX empty
    interrupt                    STA: Busy, TC, TX empty
return to i2cread after write    STA: Busy, TC, TX empty
    interrupt                    STA: Busy, STOP, TX empty
    interrupt                    STA: STOP, TX empty       *** why 2?
return to i2cread after read     STA: TX empty


 {sta = 0x1, systick = 0x4fb48, label = 0x10},
 {sta = 0x21, systick = 0x4fbcd, label = 0x0},
 {sta = 0x1, systick = 0x4fbda, label = 0x11},
call to i2cwrite,         STA: TX empty
    interrupt             STA: STOP, TX empty
return into i2cwrite,     STA: TX empty


 {sta = 0x1, systick = 0x4fbde, label = 0x100},
 {sta = 0x8041, systick = 0x4fc2e, label = 0x0},
 {sta = 0x8041, systick = 0x4fc3b, label = 0x101},
 {sta = 0x8021, systick = 0x4fc45, label = 0x0},
 {sta = 0x8001, systick = 0x4fc52, label = 0x102},
 {sta = 0x21, systick = 0x4ff70, label = 0x0},
call to i2cread                   STA: TX empty
    interrupt                     STA: Busy, TC, TX empty
return to i2cread after write     STA: Busy, TC, TX empty
    interrupt                     STA: Busy, STOP, TX empty
return to i2cread after read      STA: Busy, TX empty
    interrupt                     STA: STOP, TX empty   ****


  {sta = 0x1, systick = 0x8cbd8, label = 0x10},
  {sta = 0x8001, systick = 0x8cbe1, label = 0x11},
  {sta = 0x8001, systick = 0x8cbe4, label = 0x100},
