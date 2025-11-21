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

// Minimal PG (Persistent Group) definitions for Betaflight library compatibility
// This is a placeholder implementation for RT-Thread platform

// PG_DECLARE macro - declares a persistent group variable
// In Betaflight, this is used for parameter groups stored in flash
// For RT-Thread, we can make this a no-op or simple variable declaration
#ifndef PG_DECLARE
#define PG_DECLARE(type, name) extern type name
#endif

