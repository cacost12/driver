/*******************************************************************************
*
* FILE: 
* 		ignition.h
*
* DESCRIPTION: 
* 		Contains API functions to the flight computer parachute deployment 
*       system and contintuity readings
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IGNITION_H
#define IGNITION_H

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Ignition Continuity Status */
/* IGN_CONT_STAT = bit7 | bit6 | bit5 | bit4 | bit3 | bit2 | bit1 | bit0 

   bits5-7: unused
   bit4:   Aux 2 port continuity, 1 indicates continuity between screw terminals 
   bit3:   Aux 1 port continuity, 1 indicates continuity between screw terminals 

   bit2:   Drogue parachute deployment continuity, 1 indicates continuity between 
           screw terminals
   bit1:   Main Parachute Deployment continuity, 1 indicates continuity between 
		   screw terminals
   bit0:   Switch continuity */
typedef uint8_t IGN_CONT_STATUS;

/* Ignition Status Response Code */
typedef enum IGN_STATUS
	{
	IGN_OK              ,
	IGN_SUCCESS         ,
	IGN_EMATCH_CONT_FAIL, 
	IGN_POWER_FAIL      , 
	IGN_FAIL            ,
	IGN_SWITCH_FAIL     ,
	IGN_MAIN_CONT_FAIL  ,
	IGN_MAIN_FAIL       ,
    IGN_DROGUE_CONT_FAIL, 
	IGN_DROGUE_FAIL     ,
	IGN_UNRECOGNIZED_CMD 
	} IGN_STATUS;


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Ignition continuity code bitmasks */
#define IGN_SWITCH_MASK   	    0b00000001
#define IGN_MAIN_CONT_MASK  	0b00000010
#define IGN_DROGUE_CONT_MASK 	0b00000100
#define IGN_E_CONT_MASK         0b00001000
#define IGN_SP_CONT_MASK        0b00010000
#define IGN_NOZ_CONT_MASK       0b00100000

/* Time to supply current through ignition circuits */
#define IGN_BURN_DELAY          100


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* Polls each continuity pin and sets the continuity bits in the response 
   code */
IGN_CONT_STATUS ign_get_cont_info
	(
    void
    );


/* Asserts the ignition signal to ignite the main parachute deployment ematch. 
   Returns a response code indicating if the ignition occured succesfully */
IGN_STATUS ign_deploy_main 
    (
	void
    );


/* Asserts the ignition signal to ignite the drogue parachute deployment ematch. 
   Returns a response code indicating if the ignition occured succesfully */
IGN_STATUS ign_deploy_drogue 
    (
	void
    );


/* Returns TRUE if there is continuity across the main parachute deployment 
   ematch */
bool ign_main_cont
	(
	void
	);


/* Returns TRUE if there is continuity across the drogue parachute deployment 
   ematch */
bool ign_drogue_cont
	(
	void
	);

/* Returns TRUE if there is continuity across the switch screw terminals */
bool ign_switch_cont
	(
	void
	);


#ifdef __cplusplus
}
#endif

#endif /* IGNITION_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/