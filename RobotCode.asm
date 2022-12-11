; SimpleRobotProgram.asm
; Created by Kevin Johnson
; (no copyright applied; edit freely, no attribution necessary)
; This program does basic initialization of the DE2Bot
; and provides an example of some robot control.

; Section labels are for clarity only.

ORG 0  ; Begin program at x000
;***************************************************************
;* Initialization
;***************************************************************
Init:
	; Always a good idea to make sure the robot
	; stops in the event of a reset.
	LOAD   Zero
	OUT    LVELCMD     ; Stop motors
	OUT    RVELCMD
	OUT    SONAREN     ; Disable sonar (optional)
	OUT    BEEP        ; Stop any beeping (optional)
	
	CALL   SetupI2C    ; Configure the I2C to read the battery voltage
	CALL   BattCheck   ; Get battery voltage (and end if too low).
	OUT    LCD         ; Display battery voltage (hex, tenths of volts)

WaitForSafety:
	; This loop will wait for the user to toggle SW17.  Note that
	; SCOMP does not have direct access to SW17; it only has access
	; to the SAFETY signal contained in XIO.
	IN     SWITCHES
	OUT    BEEP
	; Wait for safety switch to be toggled
	IN     XIO         ; XIO contains SAFETY signal
	AND    Mask4       ; SAFETY signal is bit 4
	JPOS   WaitForUser ; If ready, jump to wait for PB3
	IN     TIMER       ; We'll use the timer value to
	AND    Mask1       ;  blink LED17 as a reminder to toggle SW17
	SHIFT  8           ; Shift over to LED17
	OUT    XLEDS       ; LED17 blinks at 2.5Hz (10Hz/4)
	JUMP   WaitForSafety
	
WaitForUser:
	; This loop will wait for the user to press PB3, to ensure that
	; they have a chance to prepare for any movement in the main code.
	LOADI  0
	OUT    BEEP
	; Wait for user to press PB3
	IN     TIMER       ; We'll blink the LEDs above PB3
	AND    Mask1
	SHIFT  5           ; Both LEDG6 and LEDG7
	STORE  Temp        ; (overkill, but looks nice)
	SHIFT  1
	OR     Temp
	OUT    XLEDS
	IN     XIO         ; XIO contains KEYs
	AND    Mask2       ; KEY3 mask (KEY0 is reset and can't be read)
	JPOS   WaitForUser ; not ready (KEYs are active-low, hence JPOS)
	LOAD   Zero
	OUT    XLEDS       ; clear LEDs once ready to continue

;***************************************************************
;* Main code
;***************************************************************
Main: ; "Real" program starts here.
	OUT		RESETPOS    ; reset odometer in case wheels moved after programming
	CALL FillAdjMatrix
	CALL CreateRoute
	CALL RunRoute
	
	CALL Die

Mi: DW 0
MTL: LOAD Mi
	SUB NumDestinations ; Check i < NumDestinations - 1 (array size - 1)
	ADDI 1
	JPOS MEnd
	JZERO MEnd

	LOADI &HBEEF
	OUT SSEG1
	CALL Wait1
	LOADI SortedDestArray 
	ADD Mi
	OUT LCD
	STORE Ptr ; Get pointer to SortedDestArray
	ILOAD Ptr ; Load value in pointer
	OUT SSEG1 ; Display on screen
	CALL Wait1
	CALL Wait1
	LOAD Mi
	ADDI 1 ; i++
	STORE Mi
	JUMP MTL ; Jump back to top of loop
MEnd: CALL Die

; ********************************************************
; RunRoute Subroutine
; Goes through the entire SortedDestArray and moves from each destination to the next destination, until the end
; Relies on MoveDestToDest Subroutine
; ********************************************************
RunRoute: NOP

RRi: DW 0

RRTL: LOAD RRi
	SUB NumDestinations
	ADDI 1
	JPOS RREnd	; Top of for loop checking conditions
	JZERO RREnd ; Top of for loop checking conditions
		
	LOADI SortedDestArray ; Get Sorted Array
	ADD RRi ; Get destination to visit pointer
	STORE Ptr
	ILOAD Ptr ; Load destination number from pointer
	STORE Dest2Num ; Store destination number in Dest2Num
	;OUT SSEG1
	
	CALL MoveDestToDest ; Move from Dest1Num (current position) to Dest2Num
	
	LOAD Dest2Num
	STORE Dest1Num ; Store Dest2Num in Dest1Num for next loop iteration
	
	LOAD RRi ; Load i into AC
	ADDI 1	  ; i++
	STORE RRi ; Store i++
	JUMP RRTL ; Jump back to top of loop
	
RREnd: RETURN



Dest1Num: DW 0
Dest2Num: DW 0
; *****************************************************
; MoveDestToDest Subroutine
; Moves from Destination 1 to Destination 2
; Looks up Dest1Num and Dest2Num in InitDestArray table to get their coordinates
; *****************************************************
MoveDestToDest:
	LOADI 3
	STORE m16sA ; Load 3 into m16sA
;;; Getting points (a1, b1) from InitDestArray for Dest1Num
	LOAD Dest1Num
	STORE m16sB ; Load Dest1Num into m16sB
	CALL Mult16s ; 3 * Dest1Num to calculate array offset needed for InitDestArray
	LOADI InitDestArray
	ADD mres16sL ; Get x position pointer for Point 1 (Use m16sLow since high word will be 0 since we are dealing with small values)
	STORE Ptr ; Store pointer to x position of Dest1Num in InitDestArray Table
	ILOAD Ptr ; Load X value of Dest1Num
	STORE a1 ; Store X value in a1
	LOAD Ptr ; Load pointer of Dest1Num x position again
	ADDI 1 ; Add 1 to get pointer of y position of Dest1Num
	STORE Ptr ; Store pointer to y position of Dest1Num in InitDestArray Table
	ILOAD Ptr ; Load Y Value of Dest1Num
	STORE b1 ; Store Y value in b1
	;;;;;;;;;;;;;;;
	LOAD Dest2Num
	STORE m16sB ; Load Dest2Num into m16sB
	CALL Mult16s ; 3 * Dest2Num to calculate array offset needed for InitDestArray
	LOADI InitDestArray
	ADD mres16sL ; Get x position pointer for Point 2 (Use m16sLow since high word will be 0 since we are dealing with small values)
	STORE Ptr ; Store pointer to x position of Dest2Num in InitDestArray Table
	ILOAD Ptr ; Load X value of Dest2Num
	STORE a2 ; Store X value in a2
	LOAD Ptr ; Load pointer of Dest2Num x position again
	ADDI 1 ; Add 1 to get pointer of y position of Dest2Num
	STORE Ptr ; Store pointer to y position of Dest2Num in InitDestArray Table
	ILOAD Ptr ; Load Y Value of Dest2Num
	STORE b2 ; Store Y value in b2
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
	;;;;;;;;;;
	LOAD FSlow
	STORE TurnDegreesSpeedP
	LOAD RSlow
	STORE TurnDegreesSpeedN
	;;;;;;;;;;;
	
	; Turn by Target Angle - Prev Angle
	CALL DegCalc
	LOAD degValue ; Load the degrees we want to turn by (relative to x-axis)
	OUT SSEG1
	STORE Temp ; Save in temporary variable
	SUB TDPrevAngle ; Subtract by our current heading (amount we previously turned by)
	STORE degValue ; Store in degValue
	CALL TurnDegrees ; Turn to Point 2
	LOAD Temp ; Load degrees we wanted to turn by that was stored in Temp
	STORE TDPrevAngle ; Store target degrees (not including the TDPrevAngle) in TDPrevAngle
	
	OUT RESETPOS ; Reset odometry XY position and Theta
	
	CALL DistCalc
	;;;;;;;;
	LOAD FMid
	STORE MoveDistanceSpeed
	;;;;;;;;
	CALL WaitMin
	CALL MoveDistance ; Move to Point 2
	
	RETURN

;/******************************************************************
; MoveDistance Subroutine
; Moves a distance specified by the global variable MoveDistanceAmt (in inches) at a speed specified by MoveDistanceSpeed (robot units / sec)
; MoveDistanceAmt is converted within the subroutine to RobotUnits by the InchesToRobotUnits subroutine, so there is no need for any manual conversions
;/******************************************************************
MoveDistance:
	ConvertedUnits: DW 0
	LOAD distValue
	CALL InchesToRobotUnits ; Convert MoveDistanceAMT from inches to robot units
	SUB CustomFwdOffset
	STORE ConvertedUnits ; Store the target distance (robot units) in ConvertedUnits field
MoveDistanceSetSpeed:
	LOAD ConvertedUnits
	LOAD MoveDistanceSpeed
	OUT LVELCMD
	OUT RVELCMD
	IN XPos
	SUB ConvertedUnits
	JNEG MoveDistanceSetSpeed
	LOAD Zero
	OUT LVELCMD
	OUT RVELCMD
	RETURN
MoveDistanceAmt: DW 0 ; Distance (in inches) to move for MoveDistance subroutine
MoveDistanceSpeed: DW 0 ; Speed to move at for MoveDistance subroutine

TurnDegrees:
	LOAD degValue
	JNEG TDNegAngle
	LOAD TurnDegreesSpeedP
	OUT RVELCMD
	LOAD TurnDegreesSpeedN
	OUT LVELCMD
	JUMP TDCompare
TDNegAngle:
	LOAD TurnDegreesSpeedN
	OUT RVELCMD
	LOAD TurnDegreesSpeedP
	OUT LVELCMD
	IN Theta
	ADDI -180 ; Deadband value of 2 degrees
	JNEG TDNegAngle
	JZERO TDNegAngle
	IN Theta
	SUB Deg360
	SUB degValue
	SUB CustomDegOffset
	JPOS TurnDegrees
	JUMP TDStop
TDCompare: 
	IN Theta
	ADDI -180
	JPOS TurnDegrees
	JZERO TurnDegrees
	IN Theta
	SUB degValue
	ADD CustomDegOffset
	JNEG TurnDegrees
TDStop: 
	LOAD Zero
	OUT LVELCMD
	OUT RVELCMD
	LOAD degValue
	RETURN
TurnDegreesSpeedP: DW 0 ; Speed to move at for MoveDistance subroutine
TurnDegreesSpeedN: DW 0 ; Speed to move at for MoveDistance subroutine
TDPrevAngle: DW 0

;****************************************************
; Subroutine which converts inches (stored in AC) to robot units and stores the result in AC
; This approximates the conversion between inches to robot units; The proper conversion is (Inches = RobotUnits * 1.05 / 25.4),
; but there is no Floating Point Unit (FPU) to make do this floating point arithmetic, so the formula is approximated to
; Inches = RobotUnits / 25
;****************************************************
InchesToRobotUnits:
	STORE m16sA
	LOAD Zero
	ADDI 25
	STORE m16sB
	CALL Mult16s
	LOAD mres16sL
	RETURN

Die:
; Sometimes it's useful to permanently stop execution.
; This will also catch the execution if it accidentally
; falls through from above.
	LOAD   Zero         ; Stop everything.
	OUT    LVELCMD
	OUT    RVELCMD
	OUT    SONAREN
	LOAD   DEAD         ; An indication that we are dead
	OUT    SSEG2
Forever:
	JUMP   Forever      ; Do this forever.
	DEAD:  DW &HDEAD    ; Example of a "local" variable

	
;***************************************************************
;* Subroutines
;***************************************************************

; Subroutine to wait (block) for 1 second
Wait1:
	OUT    TIMER
Wloop:
	IN     LIN
	OUT    SSEG2
	IN     TIMER
	OUT    XLEDS       ; User-feedback that a pause is occurring.
	ADDI   -10         ; 1 second in 10Hz.
	JNEG   Wloop
	RETURN
	
WaitMin:
	OUT    TIMER
WMinloop:
	IN     LIN
	OUT    SSEG2
	IN     TIMER
	OUT    XLEDS       ; User-feedback that a pause is occurring.
	ADDI   -5         ; .5 seconds (5Hz)
	JNEG   WMinloop
	RETURN

; This subroutine will get the battery voltage,
; and stop program execution if it is too low.
; SetupI2C must be executed prior to this.
BattCheck:
	CALL   GetBattLvl
	JZERO  BattCheck   ; A/D hasn't had time to initialize
	SUB    MinBatt
	JNEG   DeadBatt
	ADD    MinBatt     ; get original value back
	RETURN
; If the battery is too low, we want to make
; sure that the user realizes it...
DeadBatt:
	LOAD   Four
	OUT    BEEP        ; start beep sound
	CALL   GetBattLvl  ; get the battery level
	OUT    SSEG1       ; display it everywhere
	OUT    SSEG2
	OUT    LCD
	LOAD   Zero
	ADDI   -1          ; 0xFFFF
	OUT    LEDS        ; all LEDs on
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	Load   Zero
	OUT    BEEP        ; stop beeping
	LOAD   Zero
	OUT    LEDS        ; LEDs off
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	JUMP   DeadBatt    ; repeat forever
	
; Subroutine to read the A/D (battery voltage)
; Assumes that SetupI2C has been run
GetBattLvl:
	LOAD   I2CRCmd     ; 0x0190 (write 0B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	IN     I2C_DATA    ; get the returned data
	RETURN

; Subroutine to configure the I2C for reading batt voltage
; Only needs to be done once after each reset.
SetupI2C:
	CALL   BlockI2C    ; wait for idle
	LOAD   I2CWCmd     ; 0x1190 (write 1B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD register
	LOAD   Zero        ; 0x0000 (A/D port 0, no increment)
	OUT    I2C_DATA    ; to I2C_DATA register
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	RETURN
	
; Subroutine to block until I2C device is idle
BlockI2C:
	LOAD   Zero
	STORE  Temp        ; Used to check for timeout
BI2CL:
	LOAD   Temp
	ADDI   1           ; this will result in ~0.1s timeout
	STORE  Temp
	JZERO  I2CError    ; Timeout occurred; error
	IN     I2C_RDY     ; Read busy signal
	JPOS   BI2CL       ; If not 0, try again
	RETURN             ; Else return
I2CError:
	LOAD   Zero
	ADDI   &H12C       ; "I2C"
	OUT    SSEG1
	OUT    SSEG2       ; display error message
	JUMP   I2CError
	


; Example of table definition and data fetch
Table: ; Table = [0, 1, 2]
DW 0 
DW 1 
DW 2  

INDX: DW 0 ; Index of the data to retrieve
Ptr: DW 0 ; Pointer at the correct address in the table
output: DW 0 ; the retrieved data will be stored here


;Load Table and grabs the value at index INDX and stores it in output
 
LOADI  Table
ADD    INDX
STORE  Ptr 
ILOAD  Ptr 
STORE  output  	

;******************************************************************************;
; Atan2: 4-quadrant arctangent calculation                                     ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; Original code by Team AKKA, Spring 2015.                                     ;
; Based on methods by Richard Lyons                                            ;
; Code updated by Kevin Johnson to use software mult and div                   ;
; No license or copyright applied.                                             ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; To use: store dX and dY in global variables AtanX and AtanY.                 ;
; Call Atan2                                                                   ;
; Result (angle [0,359]) is returned in AC                                     ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; Requires additional subroutines:                                             ;
; - Mult16s: 16x16->32bit signed multiplication                                ;
; - Div16s: 16/16->16R16 signed division                                       ;
; - Abs: Absolute value                                                        ;
; Requires additional constants:                                               ;
; - One:     DW 1                                                              ;
; - NegOne:  DW -1                                                              ;
; - LowByte: DW &HFF                                                           ;
;******************************************************************************;
Atan2:
	LOAD   AtanY
	CALL   Abs          ; abs(y)
	STORE  AtanT
	LOAD   AtanX        ; abs(x)
	CALL   Abs
	SUB    AtanT        ; abs(x) - abs(y)
	JNEG   A2_sw        ; if abs(y) > abs(x), switch arguments.
	LOAD   AtanX        ; Octants 1, 4, 5, 8
	JNEG   A2_R3
	CALL   A2_calc      ; Octants 1, 8
	JNEG   A2_R1n
	RETURN              ; Return raw value if in octant 1
A2_R1n: ; region 1 negative
	ADDI   360          ; Add 360 if we are in octant 8
	RETURN
A2_R3: ; region 3
	CALL   A2_calc      ; Octants 4, 5            
	ADDI   180          ; theta' = theta + 180
	RETURN
A2_sw: ; switch arguments; octants 2, 3, 6, 7 
	LOAD   AtanY        ; Swap input arguments
	STORE  AtanT
	LOAD   AtanX
	STORE  AtanY
	LOAD   AtanT
	STORE  AtanX
	JPOS   A2_R2        ; If Y positive, octants 2,3
	CALL   A2_calc      ; else octants 6, 7
	XOR    NegOne
	ADDI   1            ; negate the angle
	ADDI   270          ; theta' = 270 - theta
	RETURN
A2_R2: ; region 2
	CALL   A2_calc      ; Octants 2, 3
	XOR    NegOne
	ADDI   1            ; negate the angle
	ADDI   90           ; theta' = 90 - theta
	RETURN
A2_calc:
	; calculates R/(1 + 0.28125*R^2)
	LOAD   AtanY
	STORE  d16sN        ; Y in numerator
	LOAD   AtanX
	STORE  d16sD        ; X in denominator
	CALL   A2_div       ; divide
	LOAD   dres16sQ     ; get the quotient (remainder ignored)
	STORE  AtanRatio
	STORE  m16sA
	STORE  m16sB
	CALL   A2_mult      ; X^2
	STORE  m16sA
	LOAD   A2c
	STORE  m16sB
	CALL   A2_mult
	ADDI   256          ; 256/256+0.28125X^2
	STORE  d16sD
	LOAD   AtanRatio
	STORE  d16sN        ; Ratio in numerator
	CALL   A2_div       ; divide
	LOAD   dres16sQ     ; get the quotient (remainder ignored)
	STORE  m16sA        ; <= result in radians
	LOAD   A2cd         ; degree conversion factor
	STORE  m16sB
	CALL   A2_mult      ; convert to degrees
	STORE  AtanT
	SHIFT  -7           ; check 7th bit
	AND    One
	JZERO  A2_rdwn      ; round down
	LOAD   AtanT
	SHIFT  -8
	ADDI   1            ; round up
	RETURN
A2_rdwn:
	LOAD   AtanT
	SHIFT  -8           ; round down
	RETURN
A2_mult: ; multiply, and return bits 23..8 of result
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8            ; move high word of result up 8 bits
	STORE  mres16sH
	LOAD   mres16sL
	SHIFT  -8           ; move low word of result down 8 bits
	AND    LowByte
	OR     mres16sH     ; combine high and low words of result
	RETURN
A2_div: ; 16-bit division scaled by 256, minimizing error
	LOAD  Nine            ; loop 8 times (256 = 2^8)
	STORE  AtanT
A2_DL:
	LOAD   AtanT
	ADDI   -1
	JPOS   A2_DN        ; not done; continue shifting
	CALL   Div16s       ; do the standard division
	RETURN
A2_DN:
	STORE  AtanT
	LOAD   d16sN        ; start by trying to scale the numerator
	SHIFT  1
	XOR    d16sN        ; if the sign changed,
	JNEG   A2_DD        ; switch to scaling the denominator
	XOR    d16sN        ; get back shifted version
	STORE  d16sN
	JUMP   A2_DL
A2_DD:
	LOAD   d16sD
	SHIFT  -1           ; have to scale denominator
	STORE  d16sD
	JUMP   A2_DL
AtanX:      DW 0
AtanY:      DW 0
AtanRatio:  DW 0        ; =y/x
AtanT:      DW 0        ; temporary value
A2c:        DW 72       ; 72/256=0.28125, with 8 fractional bits
A2cd:       DW 14668    ; = 180/pi with 8 fractional bits

;*******************************************************************************
; Mult16s:  16x16 -> 32-bit signed multiplication
; Based on Booth's algorithm.
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: does not work with factor B = -32768 (most-negative number).
; To use:
; - Store factors in m16sA and m16sB.
; - Call Mult16s
; - Result is stored in mres16sH and mres16sL (high and low words).
;*******************************************************************************
Mult16s:
	LOAD  Zero
	STORE  m16sc        ; clear carry
	STORE  mres16sH     ; clear result
	LOAD  Sixteen       ; load 16 to counter
Mult16s_loop:
	STORE  mcnt16s      
	LOAD   m16sc        ; check the carry (from previous iteration)
	JZERO  Mult16s_noc  ; if no carry, move on
	LOAD   mres16sH     ; if a carry, 
	ADD    m16sA        ; add multiplicand to result H
	STORE  mres16sH
Mult16s_noc: ; no carry
	LOAD   m16sB
	AND    One          ; check bit 0 of multiplier
	STORE  m16sc        ; save as next carry
	JZERO  Mult16s_sh   ; if no carry, move on to shift
	LOAD   mres16sH     ; if bit 0 set,
	SUB    m16sA        ; subtract multiplicand from result H
	STORE  mres16sH
Mult16s_sh:
	LOAD   m16sB
	SHIFT  -1           ; shift result L >>1
	AND    c7FFF        ; clear msb
	STORE  m16sB
	LOAD   mres16sH     ; load result H
	SHIFT  15           ; move lsb to msb
	OR     m16sB
	STORE  m16sB        ; result L now includes carry out from H
	LOAD   mres16sH
	SHIFT  -1
	STORE  mres16sH     ; shift result H >>1
	LOAD   mcnt16s
	ADDI   -1           ; check counter
	JPOS   Mult16s_loop ; need to iterate 16 times
	LOAD   m16sB
	STORE  mres16sL     ; multiplier and result L shared a word
	RETURN              ; Done
c7FFF: DW &H7FFF
m16sA: DW 0 ; multiplicand
m16sB: DW 0 ; multipler
m16sc: DW 0 ; carry
mcnt16s: DW 0 ; counter
mres16sL: DW 0 ; result low
mres16sH: DW 0 ; result high

;*******************************************************************************
; Div16s:  16/16 -> 16 R16 signed division
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: results undefined if denominator = 0.
; To use:
; - Store numerator in d16sN and denominator in d16sD.
; - Call Div16s
; - Result is stored in dres16sQ and dres16sR (quotient and remainder).
; Requires Abs subroutine
;*******************************************************************************
Div16s:
	LOAD  Zero
	STORE  dres16sR     ; clear remainder result
	STORE  d16sC1       ; clear carry
	LOAD   d16sN
	XOR    d16sD
	STORE  d16sS        ; sign determination = N XOR D
	LOAD   Seventeen
	STORE  d16sT        ; preload counter with 17 (16+1)
	LOAD   d16sD
	CALL   Abs          ; take absolute value of denominator
	STORE  d16sD
	LOAD   d16sN
	CALL   Abs          ; take absolute value of numerator
	STORE  d16sN
Div16s_loop:
	LOAD   d16sN
	SHIFT  -15          ; get msb
	AND    One          ; only msb (because shift is arithmetic)
	STORE  d16sC2       ; store as carry
	LOAD   d16sN
	SHIFT  1            ; shift <<1
	OR     d16sC1       ; with carry
	STORE  d16sN
	LOAD   d16sT
	ADDI   -1           ; decrement counter
	JZERO  Div16s_sign  ; if finished looping, finalize result
	STORE  d16sT
	LOAD   dres16sR
	SHIFT  1            ; shift remainder
	OR     d16sC2       ; with carry from other shift
	SUB    d16sD        ; subtract denominator from remainder
	JNEG   Div16s_add   ; if negative, need to add it back
	STORE  dres16sR
	LOAD   One
	STORE  d16sC1       ; set carry
	JUMP   Div16s_loop
Div16s_add:
	ADD    d16sD        ; add denominator back in
	STORE  dres16sR
	LOAD   Zero
	STORE  d16sC1       ; clear carry
	JUMP   Div16s_loop
Div16s_sign:
	LOAD   d16sN
	STORE  dres16sQ     ; numerator was used to hold quotient result
	LOAD   d16sS        ; check the sign indicator
	JNEG   Div16s_neg
	RETURN
Div16s_neg:
	LOAD   dres16sQ     ; need to negate the result
	XOR    NegOne
	ADDI   1
	STORE  dres16sQ
	RETURN	
d16sN: DW 0 ; numerator
d16sD: DW 0 ; denominator
d16sS: DW 0 ; sign value
d16sT: DW 0 ; temp counter
d16sC1: DW 0 ; carry value
d16sC2: DW 0 ; carry value
dres16sQ: DW 0 ; quotient result
dres16sR: DW 0 ; remainder result

;*******************************************************************************
; Abs: 2's complement absolute value
; Returns abs(AC) in AC
; Written by Kevin Johnson.  No licence or copyright applied.
;*******************************************************************************
Abs:
	JPOS   Abs_r
	XOR    NegOne       ; Flip all bits
	ADDI   1            ; Add one (i.e. negate number)
Abs_r:
	RETURN

;*******************************************************************************
; Mod180: modulo 180
; Returns AC%180 in AC
; Written by Kevin Johnson.  No licence or copyright applied.
;*******************************************************************************	
Mod180:
	JNEG   Mod180n      ; handle negatives
Mod180p:
	ADDI   -180
	JPOS   Mod180p      ; subtract 180 until negative
	ADDI   180          ; go back positive
	RETURN
Mod180n:
	ADDI   180          ; add 180 until positive
	JNEG   Mod180n
	ADDI   -180         ; go back negative
	RETURN
	
	
;*******************************************************************************
;DegCalc
;Takes 4 points (x1,x2,y1,y2) calculates the arctan angle between the points
;Store data in varibles a1,a2,b1,b2 respectfully. 
;Arctan (Delta(Y)/Delta(X))
;stored in degValue
;*******************************************************************************
DegCalc:
	LOAD a2
	SUB a1
	STORE AtanX
	LOAD Zero
	LOAD b2
	SUB b1
	STORE AtanY
	CALL Atan2
	STORE degValue
	
	LOAD degValue
	ADDI -180
	JNEG DeCRet
	LOAD degValue
	ADDI -360
	STORE degValue

DeCRet: RETURN
a1: DW 0
a2: DW 0
b1: DW 0
b2: DW 0 
degValue: DW 0
distValue: DW 0

;*******************************************************************************
;DistCalc
;Takes 4 points (x1,x2,y1,y2) calculates the distance between the points
;Store data in varibles a1,a2,b1,b2 respectfully. 
;Stored in distValue
;*******************************************************************************
DistCalc:
LOAD a2 ; Load x2
	SUB a1 ; Load x1
	STORE L2X ; Store x2-x1 in L2x
	
	LOAD b2 ; Load y2
	SUB b1 ; Load y1
	STORE L2Y ; Store y2-y1 in L2Y
	
	LOAD L2X
	JNEG DCYZeroCheck ; Jump to checking if L2Y is zero if L2X != 0
	JPOS DCYZeroCheck ; Jump to checking if L2Y is zero if L2X != 0
	LOAD L2Y
	CALL Abs
	STORE distValue ; Load L2Y into distValue
	RETURN ; Return from function with L2Y as answer
	
DCYZeroCheck:	LOAD L2Y
	JNEG DCEstimate ; Call distance estimation if L2Y != 0 (and L2X != 0)
	JPOS DCEstimate ; Call distance estimation if L2Y != 0 (and L2X != 0)
	LOAD L2X
	CALL Abs
	STORE distValue ; Load L2X into distValue
	RETURN ; Return from function with L2X as answer
	
DCEstimate:	CALL L2Estimate
	STORE distValue
	RETURN

;*******************************************************************************
; L2Estimate:  Pythagorean distance estimation
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: this is *not* an exact function.  I think it's most wrong
; on the axes, and maybe at 45 degrees.
; To use:
; - Store X and Y offset in L2X and L2Y.
; - Call L2Estimate
; - Result is returned in AC.
; Result will be in same units as inputs.
; Requires Abs and Mult16s subroutines.
;*******************************************************************************
L2Estimate:
	; take abs() of each value, and find the largest one
	LOAD   L2X
	CALL   Abs
	STORE  L2T1
	LOAD   L2Y
	CALL   Abs
	SUB    L2T1
	JNEG   GDSwap    ; swap if needed to get largest value in X
	ADD    L2T1
CalcDist:
	; Calculation is max(X,Y)*0.961+min(X,Y)*0.406
	STORE  m16sa
	LOAD   twofoursix       ; max * 246
	STORE  m16sB
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8
	STORE  L2T2
	LOAD   mres16sL
	SHIFT  -8        ; / 256
	AND    LowByte
	OR     L2T2
	STORE  L2T3
	LOAD   L2T1
	STORE  m16sa
	LOAD   onezerofour       ; min * 104
	STORE  m16sB
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8
	STORE  L2T2
	LOAD   mres16sL
	SHIFT  -8        ; / 256
	AND    LowByte
	OR     L2T2
	ADD    L2T3     ; sum
	RETURN
GDSwap: ; swaps the incoming X and Y
	ADD    L2T1
	STORE  L2T2
	LOAD   L2T1
	STORE  L2T3
	LOAD   L2T2
	STORE  L2T1
	LOAD   L2T3
	JUMP   CalcDist
L2X:  DW 0
L2Y:  DW 0
L2T1: DW 0
L2T2: DW 0
L2T3: DW 0

;*******************************************
;* FillAdjMatrix:
;*
;* Subroutine to fill adjacency matrix of size NumDestinations based on the destinations provided in InitDestArray table
;*******************************************
FillAdjMatrix:
	FAMi: DW 0 ; i pointer for outer loop
	FAMj: DW 0 ; j pointer for outer loop
FAMCheckI: LOAD FAMi ; Checking if outer loop conditions are true or not (whether or not to end loop)
	SUB NumDestinations
	JPOS FAMEndI ; Jump to end of outer loop
	JZero FAMEndI ; Jump to end of outer loop
	LOADI 0
	STORE FAMj ; Reset j to 0 once at top of outer loop

FAMCheckJ: LOAD FAMj
	SUB NumDestinations
	JPOS FAMEndJ
	JZero FAMEndJ	

	; Begin Calculations	

	LOADI 3
	STORE m16sA ; Load 3 into m16sA
	LOAD FAMi
	STORE m16sB ; Load i into m16sB
	CALL Mult16s ; Calculate offset from start of array by multiplying index by 3 since each destination in the array is 3 words long
	LOADI InitDestArray
	ADD mres16sL ; Get x position pointer for Point 1 (Use m16sLow since high word will be 0 since we are dealing with small values)
	STORE Ptr
	ILOAD Ptr ; Get x position for Point 1 and place in AC
	STORE a1 ; Store x position for Point 1 in a1
	OUT SSEG1
	LOAD Ptr 
	ADDI 1 ; Get y position pointer for Point 1 and place in AC
	STORE Ptr
	ILOAD Ptr ; Get y position for Point 1 and place in AC
	STORE b1 ; Store y position for Point 1 in b1
	
	LOAD FAMj
	STORE m16sB
	CALL Mult16s ; Calculate offset from start of array
	LOADI InitDestArray
	ADD mres16sL ; Get x position pointer for Point 2
	STORE Ptr
	ILOAD Ptr ; Get x position for Point 2 and place in AC
	STORE a2 ; Store x position pointer for Point 2 in a2
	OUT SSEG2
	LOAD Ptr 
	ADDI 1 ; Get y position pointer for Point 2 and place in AC
	STORE Ptr
	ILOAD Ptr ; Get y position for Point 2 and place in AC
	STORE b2 ; Store y position for Point 2 in b1
	
	LOAD NumDestinations
	STORE m16sA ; Load NumDestinations into m16sA
	LOAD FAMi
	STORE m16sB ; Load i into m16sB
	CALL Mult16s ; Multiply NumDestinations by i
	LOAD mres16sL ; Load result of NumDestinations*i into AC
	ADD FAMj ; Add j to result
	STORE Temp ; Temp holds offset needed for a 2D matrix [i][j] ; In this case, the offset is NumDestinations*i + j
	CALL DistCalc ; Calculate distance between two points
	LOADI AdjMatrixDist ; Get base pointer to AdjMatrixDist
	ADD Temp ; Get proper index pointer to write to in AdjMatrixDist
	STORE Ptr ; Store AdjMatrixDist pointer in Ptr
	OUT LCD
	LOAD distValue ; Load distance between two points into AC
	;OUT SSEG1
	;CALL Wait1
	;CALL Wait1
	;LOADI &HBEEF
	;OUT SSEG1
	;CALL Wait1
	;LOAD distValue
	ISTORE Ptr ; Write distance to AdjMatrixDist array
	CALL DegCalc ; Calculate angle between two points
	LOADI AdjMatrixAng ; Get base pointer to AdjMatrixAng
	ADD Temp ; Get proper index pointer to write to in AdjMatrixAng
	STORE Ptr ; Store AdjMatrixAng pointer in Ptr
	LOAD degValue ; Load angle between two points into AC
	ISTORE Ptr ; Write angle to AdjMatrixAng array
	
	; End Calculations
    
    LOAD FAMj
    ADDI 1
    STORE FAMj
    JUMP FAMCheckJ
    
FAMEndJ: LOAD FAMi
	ADDI 1
	STORE FAMi
	JUMP FAMCheckI
	
FAMEndI: RETURN



;*******************************************
;* CreateRoute
;*
;* Subroutine to generate a route between destinations in InitDestArray based on the AdjMatrixDist array
;*******************************************
CreateRoute:
closestIdx: DW -1
closestDist: DW 50000
CRi: DW 0
CRj: DW 0
currDest: DW 0 ; 0 as we will start at Destination 0 (origin)
visitedSet: DW 1

CRCheckI: LOAD CRi
	SUB NumDestinations
	ADDI 1
	JPOS CREndI
	JZERO CREndI
	
	LOADI 50000
	STORE closestDist
	
	LOADI 0
	STORE CRj

CRCheckJ: LOAD CRj
	SUB NumDestinations
	JPOS CREndJ
	JZERO CREndJ
	
	; Start Calculations
	
	LOAD CRj
    SUB currDest
    JZERO IfCheckEnd ; (if j == currDest) then skip to IfCheckEnd
    LOAD NumDestinations
    STORE m16sA
    LOAD currDest
    STORE m16sB
    CALL Mult16s ; Multiply NumDestinations*currDest
    LOAD mres16sL
    ADD CRj		; Get pointer offset for Adj[currDest][j]
    STORE Temp  ; Store pointer offset in Temp
    LOADI AdjMatrixDist
    ADD Temp
    STORE Ptr ; Store pointer for Adj[currDest][j] in Ptr
    ILOAD Ptr ; Load value of Adj[currDest][j] into AC
    SUB closestDist
    JPOS IfCheckEnd ; Branch to IfCheckEnd if (Adj[currDestination][j] >= closest)]
    JZERO IfCheckEnd ; Branch to IfCheckEnd if (Adj[currDestination][j] >= closest)]
    
    LOADI Mask0 ; Load pointer to Mask0
    ADD CRj ; Add j to get pointer for mask shifted left by j bits
    STORE Temp ; Store pointer for mask in Temp
    ILOAD Temp ; Load mask (1 << j)
    AND visitedSet
    JNEG IfCheckEnd ; Branch to IfCheckEnd if (visistedSet & (1 << j) != 0)
    JPOS IfCheckEnd ; Branch to IfCheckEnd if (visitedSet & (1 << j) != 0)
    ILOAD Ptr ; Load Adj[currDest][j] into AC
    STORE closestDist ; closestDist = Adj[currDest][j]
    LOAD CRj
    STORE closestIdx ; closestIdx = j
    LOADI SortedDestArray
    ADD CRi
    STORE Ptr ; Store SortedDestArray[i] pointer in Ptr
    LOAD closestIdx ; Load closestIdx in AC
    ISTORE Ptr ; Load closestIdx into SortedDestArray[i]
    
IfCheckEnd: NOP 
    ; End Calculations
    
    LOAD CRj ; Increment j at the end of the loop and jump back to top of loop
    ADDI 1
    STORE CRj
    JUMP CRCheckJ
    
CREndJ: LOADI Mask0 ; Load pointer to mask for 0b000000001
	ADD closestIdx ; Shift left by adding closestIdx
	STORE Temp
	ILOAD Temp ; Load mask (1 << closestIdx)
	OR visitedSet ; OR by visitedSet
	STORE visitedSet ; Store visited set by result in AC
	LOAD closestIdx ; Load closestIdx
	STORE currDest ; Set current destination (currDest) to closestIdx
	
	LOAD CRi ; Increment i at the end of the loop and jump back to the top of the loop
	ADDI 1
	STORE CRi
	JUMP CRCheckI
	
CREndI: RETURN



;***************************************************************
;* Variables
;***************************************************************

Temp:  DW 0 ; "Temp" is not a great name, but can be useful
Temp2: DW 0
Temp3: DW 0
CDX: DW 0      ; current desired X
CDY: DW 0      ; current desired Y
CDT: DW 0      ; current desired angle
CX:  DW 0      ; sampled X
CY:  DW 0      ; sampled Y
CT:  DW 0      ; sampled theta

xDest:	DW 0
yDest:	DW 0
Order:  DW 0
currT:	DW 0
AngErr:	DW 0
posiErr: DW 0
dist:	DW 0
currX:	DW 0
currY:	DW 0
;List of needed pointers
Turn: DW 0
PtrX: DW 0
PtrY: DW 0
PtrOrder:  DW 0


;***************************************************************
;* Constants
;* (though there is nothing stopping you from writing to these)
;***************************************************************
Neg45:		DW -45
NegEleven:	DW -11
negfive:	DW -5
NegOne:   DW -1
NegOneHalf:	DW -.5
Zero:     DW 0
OneHalf:	DW .5
One:      DW 1
Two:      DW 2
Three:    DW 3
Four:     DW 4
Five:     DW 5
Six:      DW 6
Seven:    DW 7
Eight:    DW 8
Nine:     DW 9
Ten:      DW 10
Eleven:	  DW 11
Sixteen:	  DW 16
Seventeen: DW 17
FortyFive:	DW 45
Fifty:	  DW 50
OneHundred:	DW 100
onezerofour: DW 104
twofoursix: DW 246
yintercept:	DW 1480

; Some bit masks.
; Masks of multiple bits can be constructed by ORing these
; 1-bit masks together.
Mask0:    DW &B00000001
Mask1:    DW &B00000010
Mask2:    DW &B00000100
Mask3:    DW &B00001000
Mask4:    DW &B00010000
Mask5:    DW &B00100000
Mask6:    DW &B01000000
Mask7:    DW &B10000000
Mask8:	  DW &H100
Mask9:    DW &H200
Mask10:	  DW &H400
Mask11:   DW &H800
Mask12:   DW &H1000
Mask13:   DW &H2000
Mask14:   DW &H4000
Mask15:   DW &H8000
LowByte:  DW &HFF      ; binary 00000000 1111111
LowNibl:  DW &HF       ; 0000 0000 0000 1111
HighWord: DW &HFFFF

; some useful movement values
OneMeter: DW 952       ; ~1m in 1.05mm units
HalfMeter: DW 476      ; ~0.5m in 1.05mm units
OneFoot:  DW 290       ; ~1ft in 1.05mm robot units
TwoFeet:  DW 581       ; ~2ft in 1.05mm units
TwoFeetInches: DW 24
Deg90:    DW 90        ; 90 degrees in odometer units
Deg180:   DW 180       ; 180
Deg270:   DW 270       ; 270
Deg360:   DW 360       ; can never actually happen; for math only
FSlow:    DW 100       ; 100 is about the lowest velocity value that will move
RSlow:    DW -100
FMid:     DW 350       ; 350 is a medium speed
RMid:     DW -350
FFast:    DW 500       ; 500 is almost max speed (511 is max)
RFast:    DW -500

SlowFwdOffset: DW 9 ; Robot units
SlowDegOffset: DW 5

MidFwdOffset: DW 119 ; Robot units
MidDegOfset: DW 60

FastFwdOffset: DW 244 ; Robot units
FastDegOffset: DW 123

CustomFwdOffset: DW 119 ; Robot units (Vel^2 / 1024)
CustomDegOffset: DW 60 ; Robot units (Vel^2 / 2030)

MinBatt:  DW 110       ; 13.0V - minimum safe battery voltage
I2CWCmd:  DW &H1190    ; write one i2c byte, read one byte, addr 0x90
I2CRCmd:  DW &H0190    ; write nothing, read one byte, addr 0x90

;***************************************************************
;* IO address space map
;***************************************************************
SWITCHES: EQU &H00  ; slide switches
LEDS:     EQU &H01  ; red LEDs
TIMER:    EQU &H02  ; timer, usually running at 10 Hz
XIO:      EQU &H03  ; pushbuttons and some misc. inputs
SSEG1:    EQU &H04  ; seven-segment display (4-digits only)
SSEG2:    EQU &H05  ; seven-segment display (4-digits only)
LCD:      EQU &H06  ; primitive 4-digit LCD display
XLEDS:    EQU &H07  ; Green LEDs (and Red LED16+17)
BEEP:     EQU &H0A  ; Control the beep
CTIMER:   EQU &H0C  ; Configurable timer for interrupts
LPOS:     EQU &H80  ; left wheel encoder position (read only)
LVEL:     EQU &H82  ; current left wheel velocity (read only)
LVELCMD:  EQU &H83  ; left wheel velocity command (write only)
RPOS:     EQU &H88  ; same values for right wheel...
RVEL:     EQU &H8A  ; ...
RVELCMD:  EQU &H8B  ; ...
I2C_CMD:  EQU &H90  ; I2C module's CMD register,
I2C_DATA: EQU &H91  ; ... DATA register,
I2C_RDY:  EQU &H92  ; ... and BUSY register
UART_DAT: EQU &H98  ; UART data
UART_RDY: EQU &H98  ; UART status
SONAR:    EQU &HA0  ; base address for more than 16 registers....
DIST0:    EQU &HA8  ; the eight sonar distance readings
DIST1:    EQU &HA9  ; ...
DIST2:    EQU &HAA  ; ...
DIST3:    EQU &HAB  ; ...
DIST4:    EQU &HAC  ; ...
DIST5:    EQU &HAD  ; ...
DIST6:    EQU &HAE  ; ...
DIST7:    EQU &HAF  ; ...
SONALARM: EQU &HB0  ; Write alarm distance; read alarm register
SONARINT: EQU &HB1  ; Write mask for sonar interrupts
SONAREN:  EQU &HB2  ; register to control which sonars are enabled
XPOS:     EQU &HC0  ; Current X-position (read only)
YPOS:     EQU &HC1  ; Y-position
THETA:    EQU &HC2  ; Current rotational position of robot (0-359)
RESETPOS: EQU &HC3  ; write anything here to reset odometry to 0
RIN:      EQU &HC8
LIN:      EQU &HC9

NumDestinations: DW 4
;***************************************************************
;* Array to hold the initial given destinations locations and destination number
;  3 words are reserved for each destination
;  Word 1 = X Position
;  Word 2 = Y Position
;  Word 3 = Destination Number
;  Therefore, MEM(0) to MEM(2) (inclusive) represent data for the first destination, MEM(3) to MEM(5) represent data for the second destination, etc.
;***************************************************************
InitDestArray: DW 0 ; Dest0 X (Origin)
DW  0 ; Dest0 Y (Origin)
DW  0 	; Dest0 # (Origin)

DW  24 	; Dest1 X
DW  0 	; Dest1 Y
DW  1 	; Dest1 #

DW  24 	; Dest2 X
DW  24	; Dest2 Y
DW  2 	; Dest2 #

DW  36 	; Dest3 X
DW  24 ; Dest3 Y
DW  3 	; Dest3 #

DW  24 	; Dest4 X
DW  0 	; Dest4 Y
DW  4 	; Dest4 #

DW  30 	; Dest5 X
DW  0 	; Dest5 Y
DW  5 	; Dest5 #

DW  0	; Dest6 X
DW  0	; Dest6 Y
DW  0	; Dest6 #

DW  0 	; Dest7 X
DW  0	; Dest7 Y
DW  0	; Dest7 #

DW  0	; Dest8 X
DW  0	; Dest8 Y
DW  0	; Dest8 #

DW  0	; Dest9 X
DW  0	; Dest9 Y
DW  0	; Dest9 #

DW  0	; Dest10 X
DW  0	; Dest10 Y
DW  0	; Dest10 #

DW  0	; Dest11 X
DW  0	; Dest11 Y
DW  0	; Dest11 #

DW  0	; Dest12 X
DW  0	; Dest12 Y
DW  0	; Dest12 #

;***************************************************************
;* Array for final destinations, which holds the order of destinations to go to after Greedy Algorithm has been executed
;***************************************************************
SortedDestArray: DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0

;***************************************************************
;* Array to hold the distances to each destination for the adjacency matrix (13x13)
;***************************************************************
AdjMatrixDist: DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  0
DW  41
DW  42
DW  43
DW  44
DW  45
DW  46
DW  47
DW  48
DW  49
DW  50
DW  51
DW  52
DW  53
DW  54
DW  55
DW  56
DW  57
DW  58
DW  59
DW  60
DW  61
DW  62
DW  63
DW  64
DW  65
DW  66
DW  67
DW  68
DW  69
DW  70
DW  71
DW  72
DW  73
DW  74
DW  75
DW  76
DW  77
DW  78
DW  79
DW  80
DW  81
DW  82
DW  83
DW  84
DW  85
DW  86
DW  87
DW  88
DW  89
DW  90
DW  91
DW  92
DW  93
DW  94
DW  95
DW  96
DW  97
DW  98
DW  99
DW  100
DW  101
DW  102
DW  103
DW  104
DW  105
DW  106
DW  107
DW  108
DW  109
DW  110
DW  111
DW  112
DW  113
DW  114
DW  115
DW  116
DW  117
DW  118
DW  119
DW  120
DW  121
DW  122
DW  123
DW  124
DW  125
DW  126
DW  127
DW  128
DW  129
DW  130
DW  131
DW  132
DW  133
DW  134
DW  135
DW  136
DW  137
DW  138
DW  139
DW  140
DW  141
DW  142
DW  143
DW  144
DW  145
DW  146
DW  147
DW  148
DW  149
DW  150
DW  151
DW  152
DW  153
DW  154
DW  155
DW  156
DW  157
DW  158
DW  159
DW  160
DW  161
DW  162
DW  163
DW  164
DW  165
DW  166
DW  167
DW  168

;***************************************************************
;* Array to hold the angles to each destination for the adjacency matrix
;***************************************************************
AdjMatrixAng: DW  0
DW  1
DW  2
DW  3
DW  4
DW  5
DW  6
DW  7
DW  8
DW  9
DW  10
DW  11
DW  12
DW  13
DW  14
DW  15
DW  16
DW  17
DW  18
DW  19
DW  20
DW  21
DW  22
DW  23
DW  24
DW  25
DW  26
DW  27
DW  28
DW  29
DW  30
DW  31
DW  32
DW  33
DW  34
DW  35
DW  36
DW  37
DW  38
DW  39
DW  40
DW  41
DW  42
DW  43
DW  44
DW  45
DW  46
DW  47
DW  48
DW  49
DW  50
DW  51
DW  52
DW  53
DW  54
DW  55
DW  56
DW  57
DW  58
DW  59
DW  60
DW  61
DW  62
DW  63
DW  64
DW  65
DW  66
DW  67
DW  68
DW  69
DW  70
DW  71
DW  72
DW  73
DW  74
DW  75
DW  76
DW  77
DW  78
DW  79
DW  80
DW  81
DW  82
DW  83
DW  84
DW  85
DW  86
DW  87
DW  88
DW  89
DW  90
DW  91
DW  92
DW  93
DW  94
DW  95
DW  96
DW  97
DW  98
DW  99
DW  100
DW  101
DW  102
DW  103
DW  104
DW  105
DW  106
DW  107
DW  108
DW  109
DW  110
DW  111
DW  112
DW  113
DW  114
DW  115
DW  116
DW  117
DW  118
DW  119
DW  120
DW  121
DW  122
DW  123
DW  124
DW  125
DW  126
DW  127
DW  128
DW  129
DW  130
DW  131
DW  132
DW  133
DW  134
DW  135
DW  136
DW  137
DW  138
DW  139
DW  140
DW  141
DW  142
DW  143
DW  144
DW  145
DW  146
DW  147
DW  148
DW  149
DW  150
DW  151
DW  152
DW  153
DW  154
DW  155
DW  156
DW  157
DW  158
DW  159
DW  160
DW  161
DW  162
DW  163
DW  164
DW  165
DW  166
DW  167
DW  168
	  
;***************************************************************
;* LCD Display (DE##)
;***************************************************************

HZero:	  DW &HDE00	   
HOne:	  DW &HDE01
HTwo:	  DW &HDE02
HThree:	  DW &HDE03
HFour:	  DW &HDE04
HFive:	  DW &HDE05
HSix:	  DW &HDE06
HSeven:	  DW &HDE07
HEight:	  DW &HDE08
HNine:	  DW &HDE09
HTen:	  DW &HDE10
HEleven:  DW &HDE11
HTwelve:  DW &HDE12


LoadDE00:
	LOAD HZero
	OUT  LCD
	RETURN
	
LoadDE01:
	LOAD HOne
	OUT  LCD
	RETURN
	
LoadDE02:
	LOAD HTwo
	OUT  LCD
	RETURN
	
LoadDE03:
	LOAD HThree
	OUT  LCD
	RETURN
	
LoadDE04:
	LOAD HFour
	OUT  LCD
	RETURN
	
LoadDE05:
	LOAD HFive
	OUT  LCD
	RETURN
	
LoadDE06:
	LOAD HSix
	OUT  LCD
	RETURN
	
LoadDE07:
	LOAD HSeven
	OUT  LCD
	RETURN
	
LoadDE08:
	LOAD HEight
	OUT  LCD
	RETURN
	
LoadDE09:
	LOAD HNine
	OUT  LCD
	RETURN
	
LoadDE10:
	LOAD HTen
	OUT  LCD
	RETURN
	
LoadDE11:
	LOAD HEleven
	OUT  LCD
	RETURN
	
LoadDE12:
	LOAD HTwelve
	OUT  LCD
	RETURN