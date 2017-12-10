/*
** =============================================================================
**
** File: ImmVibeSPI.c
**
** Description:
**     Device-dependent functions called by Immersion TSP API
**     to control PWM duty cycle, amp enable/disable, save IVT file, etc...
**
** $Revision$
**
** Copyright (c) 2007-2013 Immersion Corporation. All Rights Reserved.
**
** This file contains Original Code and/or Modifications of Original Code
** as defined in and that are subject to the GNU Public License v2 -
** (the 'License'). You may not use this file except in compliance with the
** License. You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES,
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see
** the License for the specific language governing rights and limitations
** under the License.
**
** =============================================================================
*/

#ifdef IMMVIBESPIAPI
#undef IMMVIBESPIAPI
#endif
#define IMMVIBESPIAPI static

/*
** This SPI supports only one actuator.
*/
#error Please Set NUM_ACTUATORS to the number of actuators supported by this SPI.
#define NUM_ACTUATORS       1

/*
** Called to disable amp (disable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex)
{
#error Please implement

    /* Disable amp */
    /* To be implemented with appropriate hardware access macros */

    return VIBE_S_SUCCESS;
}

/*
** Called to enable amp (enable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable(VibeUInt8 nActuatorIndex)
{
#error Please implement

    /* Reset PWM frequency */
    /* To be implemented with appropriate hardware access macros */

    /* Set duty cycle to 50% */
    /* To be implemented with appropriate hardware access macros */

    /* Enable amp */
    /* To be implemented with appropriate hardware access macros */

    return VIBE_S_SUCCESS;
}

/*
** Called at initialization time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize(void)
{
    /* Disable amp */
    ImmVibeSPI_ForceOut_AmpDisable(0);

#error Please implement

    /* Set PWM frequency */
    /* To be implemented with appropriate hardware access macros */

    /* Set duty cycle to 50% */
    /* To be implemented with appropriate hardware access macros */

    return VIBE_S_SUCCESS;
}

/*
** Called at termination time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate(void)
{
#error Please implement

    /* Disable amp */
    ImmVibeSPI_ForceOut_AmpDisable(0);

    /* Set PWM frequency */
    /* To be implemented with appropriate hardware access macros */

    /* Set duty cycle to 50% */
    /* To be implemented with appropriate hardware access macros */

    return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set force output, and enable amp if required
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
#error Please implement

    /* 
    ** For LRA:
    **      nBufferSizeInBytes should be equal to 1 if nOutputSignalBitDepth is equal to 8
    **      nBufferSizeInBytes should be equal to 2 if nOutputSignalBitDepth is equal to 16
    */

    /* Below based on assumed 8 bit PWM, other implementation are possible */

    /* M = 1, N = 256, 1 <= nDutyCycle <= (N -M) */

    /* Output force: nForce is mapped from [-127, 127] to [1, 255] */
    /* To be implemented with appropriate hardware access macros */

    return VIBE_S_SUCCESS;
}

/*
** Called to set force output frequency parameters
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetFrequency(VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID, VibeUInt32 nFrequencyParameterValue)
{
#error Please review the code between the #if and #endif

#if 0 
    /* 
    ** The following code is provided as sample. If enabled, it will allow device 
    ** frequency parameters tuning via the ImmVibeSetDeviceKernelParameter API.
    ** Please modify as required. 
    */
    switch (nFrequencyParameterID)
    {
        case VIBE_KP_CFG_FREQUENCY_PARAM1:
            /* Update frequency parameter 1 */
            break;

        case VIBE_KP_CFG_FREQUENCY_PARAM2:
            /* Update frequency parameter 2 */
            break;

        case VIBE_KP_CFG_FREQUENCY_PARAM3:
            /* Update frequency parameter 3 */
            break;

        case VIBE_KP_CFG_FREQUENCY_PARAM4:
            /* Update frequency parameter 4 */
            break;

        case VIBE_KP_CFG_FREQUENCY_PARAM5:
            /* Update frequency parameter 5 */
            break;

        case VIBE_KP_CFG_FREQUENCY_PARAM6:
            /* Update frequency parameter 6 */
            break;
    }
#endif

    return VIBE_S_SUCCESS;
}

/*
** Called to save an IVT data file (pIVT) to a file (szPathName)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_IVTFile_Save(const VibeUInt8 *pIVT, VibeUInt32 nIVTSize, const char *szPathname)
{
#error Please implement

    /* To be implemented */

    return VIBE_S_SUCCESS;
}

/*
** Called to delete an IVT file
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_IVTFile_Delete(const char *szPathname)
{
#error Please implement

    /* To be implemented */

    return VIBE_S_SUCCESS;
}

/*
** Called to get the device name (device name must be returned as ANSI char)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName(VibeUInt8 nActuatorIndex, char *szDevName, int nSize)
{
#error Please review the code between the #if and #endif

#if 0   /* The following code is provided as sample. Please modify as required. */
    if ((!szDevName) || (nSize < 1)) return VIBE_E_FAIL;

    strncpy(szDevName, "Generic", nSize-1);
    szDevName[nSize - 1] = '\0';    /* make sure the string is NULL terminated */
#endif

    return VIBE_S_SUCCESS;
}

/*
** Called at initialization time to get the number of actuators
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetNum(void)
{
    return NUM_ACTUATORS;
}
