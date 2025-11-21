/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

// Platform-specific definitions for Betaflight library compatibility
// This is a minimal implementation for RT-Thread platform

// FAST_CODE macros - used to mark functions for fast execution
// For RT-Thread, we define them as empty (no special memory placement)
#ifndef FAST_CODE
#define FAST_CODE
#endif

#ifndef FAST_CODE_NOINLINE
#define FAST_CODE_NOINLINE
#endif

// UNUSED macro for unused parameters
#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

// FAST_DATA_ZERO_INIT - used to mark static data that should be zero-initialized
// For RT-Thread, we define it as empty (compiler handles zero-initialization)
#ifndef FAST_DATA_ZERO_INIT
#define FAST_DATA_ZERO_INIT
#endif

// Use RT-Thread's itoa function instead of Betaflight's implementation
#ifndef HAVE_ITOA_FUNCTION
#define HAVE_ITOA_FUNCTION
#endif

