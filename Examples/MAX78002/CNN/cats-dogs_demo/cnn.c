/*******************************************************************************
* Copyright (C) 2019-2022 Maxim Integrated Products, Inc., All rights Reserved.
*
* This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************/

// cats-dogs
// Created using ai8xize.py --test-dir sdk/Examples/MAX78002/CNN --prefix cats-dogs --checkpoint-file trained/ai85-catsdogs-qat8-q.pth.tar --config-file networks/cats-dogs-chw.yaml --softmax --device MAX78002 --compact-data --mexpress --timer 0 --display-checkpoint --verbose

// DO NOT EDIT - regenerate this file instead!

// Configuring 7 layers
// Input data: CHW
// Layer 0: 3x64x64, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 15x64x64 output
// Layer 1: 15x64x64, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 30x32x32 output
// Layer 2: 30x32x32, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 60x16x16 output
// Layer 3: 60x16x16, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 30x8x8 output
// Layer 4: 30x8x8, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 30x4x4 output
// Layer 5: 30x4x4, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 30x4x4 output
// Layer 6: 30x4x4 flattened to 480x1x1, no pooling, linear with kernel size 1x1, stride 1/1, pad 0/0, no activation, 2x1x1 output

#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "mxc.h"
#include "gcfr_regs.h"
#include "cnn.h"
#include "weights.h"

void CNN_ISR(void)
{
  // Acknowledge interrupt to all quadrants
  *((volatile uint32_t *) 0x51000000) &= ~((1<<12) | 1);
  *((volatile uint32_t *) 0x52000000) &= ~((1<<12) | 1);
  *((volatile uint32_t *) 0x53000000) &= ~((1<<12) | 1);
  *((volatile uint32_t *) 0x54000000) &= ~((1<<12) | 1);

  CNN_COMPLETE; // Signal that processing is complete
#ifdef CNN_INFERENCE_TIMER
  cnn_time = MXC_TMR_SW_Stop(CNN_INFERENCE_TIMER);
#else
  cnn_time = 1;
#endif
}

int cnn_continue(void)
{
  cnn_time = 0;

  *((volatile uint32_t *) 0x51000000) |= 1; // Re-enable quadrant 0

  return CNN_OK;
}

int cnn_stop(void)
{
  *((volatile uint32_t *) 0x51000000) &= ~1; // Disable quadrant 0

  return CNN_OK;
}

void memcpy32(uint32_t *dst, const uint32_t *src, int n)
{
  while (n-- > 0) {
    *dst++ = *src++;
  }
}

static const uint32_t kernels[] = KERNELS;

int cnn_load_weights(void)
{
  uint32_t len;
  volatile uint32_t *addr;
  const uint32_t *ptr = kernels;

  while ((addr = (volatile uint32_t *) *ptr++) != 0) {
    *((volatile uint8_t *) ((uint32_t) addr | 1)) = 0x01; // Set address
    len = *ptr++;
    while (len-- > 0)
      *addr++ = *ptr++;
  }

  return CNN_OK;
}

static const uint8_t bias_0[] = BIAS_0;

static void memcpy_8to32(uint32_t *dst, const uint8_t *src, int n)
{
  while (n-- > 0) {
    *dst++ = *src++;
  }
}

int cnn_load_bias(void)
{
  memcpy_8to32((uint32_t *) 0x51180000, bias_0, sizeof(uint8_t) * 2);

  return CNN_OK;
}

int cnn_init(void)
{
  *((volatile uint32_t *) 0x51000000) = 0x00000008; // Enable clocks
  *((volatile uint32_t *) 0x52000000) = 0x00000008; // Enable clocks
  *((volatile uint32_t *) 0x53000000) = 0x00000008; // Enable clocks
  *((volatile uint32_t *) 0x54000000) = 0x00000008; // Enable clocks
  *((volatile uint32_t *) 0x50001000) = 0x00000000; // AON control
  *((volatile uint32_t *) 0x51000004) = 0x0000040e; // SRAM control
  *((volatile uint32_t *) 0x52000004) = 0x0000040e; // SRAM control
  *((volatile uint32_t *) 0x53000004) = 0x0000040e; // SRAM control
  *((volatile uint32_t *) 0x54000004) = 0x0000040e; // SRAM control
  *((volatile uint32_t *) 0x5100000c) = 0x00001c80; // Clear registers
  *((volatile uint32_t *) 0x5200000c) = 0x00001c80; // Clear registers
  *((volatile uint32_t *) 0x5300000c) = 0x00001c80; // Clear registers
  *((volatile uint32_t *) 0x5400000c) = 0x00001c80; // Clear registers
  while ((*((volatile uint32_t *) 0x5100000c) & 0x2000000) != 0x2000000); // Wait for clear
  while ((*((volatile uint32_t *) 0x5200000c) & 0x2000000) != 0x2000000); // Wait for clear
  while ((*((volatile uint32_t *) 0x5300000c) & 0x2000000) != 0x2000000); // Wait for clear
  while ((*((volatile uint32_t *) 0x5400000c) & 0x2000000) != 0x2000000); // Wait for clear
  *((volatile uint32_t *) 0x5100000c) = 0x00000000; // Reset BIST
  *((volatile uint32_t *) 0x5200000c) = 0x00000000; // Reset BIST
  *((volatile uint32_t *) 0x5300000c) = 0x00000000; // Reset BIST
  *((volatile uint32_t *) 0x5400000c) = 0x00000000; // Reset BIST

  *((volatile uint32_t *) 0x51000000) = 0x00100008; // Stop SM
  *((volatile uint32_t *) 0x51000008) = 0x00000006; // Layer count
  *((volatile uint32_t *) 0x52000000) = 0x00100008; // Stop SM
  *((volatile uint32_t *) 0x52000008) = 0x00000006; // Layer count
  *((volatile uint32_t *) 0x53000000) = 0x00100008; // Stop SM
  *((volatile uint32_t *) 0x53000008) = 0x00000006; // Layer count
  *((volatile uint32_t *) 0x54000000) = 0x00100008; // Stop SM
  *((volatile uint32_t *) 0x54000008) = 0x00000006; // Layer count

  return CNN_OK;
}

int cnn_configure(void)
{
  // Layer 0 quadrant 0
  *((volatile uint32_t *) 0x51100004) = 0x0001803f; // Rows
  *((volatile uint32_t *) 0x51100008) = 0x0001803f; // Columns
  *((volatile uint32_t *) 0x51100018) = 0x00000010; // Stride
  *((volatile uint32_t *) 0x5110001c) = 0x00048000; // SRAM write ptr
  *((volatile uint32_t *) 0x51100024) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x51100030) = 0x00886b60; // Layer control
  *((volatile uint32_t *) 0x51100034) = 0x00070000; // Layer control 2
  *((volatile uint32_t *) 0x51100038) = 0x00009ff0; // Mask count
  *((volatile uint32_t *) 0x5110003c) = 0x00009f80; // Mask offset
  *((volatile uint32_t *) 0x51100040) = 0x0000000e; // Output channel count
  *((volatile uint32_t *) 0x51100044) = 0x0000003f; // TRAM ptr max
  *((volatile uint32_t *) 0x5110004c) = 0x00024000; // Post processing register
  *((volatile uint32_t *) 0x51100048) = 0x00010001; // Mask and processor enables

  // Layer 0 quadrant 1
  *((volatile uint32_t *) 0x52100004) = 0x0001803f; // Rows
  *((volatile uint32_t *) 0x52100008) = 0x0001803f; // Columns
  *((volatile uint32_t *) 0x52100018) = 0x00000010; // Stride
  *((volatile uint32_t *) 0x5210001c) = 0x00048000; // SRAM write ptr
  *((volatile uint32_t *) 0x52100024) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x52100030) = 0x00880b60; // Layer control
  *((volatile uint32_t *) 0x52100034) = 0x00070000; // Layer control 2
  *((volatile uint32_t *) 0x52100038) = 0x00009ff0; // Mask count
  *((volatile uint32_t *) 0x5210003c) = 0x00009f80; // Mask offset
  *((volatile uint32_t *) 0x52100040) = 0x0000000e; // Output channel count
  *((volatile uint32_t *) 0x52100044) = 0x0000003f; // TRAM ptr max
  *((volatile uint32_t *) 0x5210004c) = 0x00024000; // Post processing register
  *((volatile uint32_t *) 0x52100048) = 0x00010001; // Mask and processor enables

  // Layer 0 quadrant 2
  *((volatile uint32_t *) 0x53100004) = 0x0001803f; // Rows
  *((volatile uint32_t *) 0x53100008) = 0x0001803f; // Columns
  *((volatile uint32_t *) 0x53100018) = 0x00000010; // Stride
  *((volatile uint32_t *) 0x5310001c) = 0x00048000; // SRAM write ptr
  *((volatile uint32_t *) 0x53100024) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x53100030) = 0x00880b60; // Layer control
  *((volatile uint32_t *) 0x53100034) = 0x00070000; // Layer control 2
  *((volatile uint32_t *) 0x53100038) = 0x00009ff0; // Mask count
  *((volatile uint32_t *) 0x5310003c) = 0x00009f80; // Mask offset
  *((volatile uint32_t *) 0x53100040) = 0x0000000e; // Output channel count
  *((volatile uint32_t *) 0x53100044) = 0x0000003f; // TRAM ptr max
  *((volatile uint32_t *) 0x5310004c) = 0x00024000; // Post processing register
  *((volatile uint32_t *) 0x53100048) = 0x00010001; // Mask and processor enables

  // Layer 0 quadrant 3
  *((volatile uint32_t *) 0x54100004) = 0x0001803f; // Rows
  *((volatile uint32_t *) 0x54100008) = 0x0001803f; // Columns
  *((volatile uint32_t *) 0x54100018) = 0x00000010; // Stride
  *((volatile uint32_t *) 0x5410001c) = 0x00048000; // SRAM write ptr
  *((volatile uint32_t *) 0x54100024) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x54100030) = 0x00880b60; // Layer control
  *((volatile uint32_t *) 0x54100034) = 0x00070000; // Layer control 2
  *((volatile uint32_t *) 0x54100038) = 0x00009ff0; // Mask count
  *((volatile uint32_t *) 0x5410003c) = 0x00009f80; // Mask offset
  *((volatile uint32_t *) 0x54100040) = 0x0000000e; // Output channel count
  *((volatile uint32_t *) 0x54100044) = 0x0000003f; // TRAM ptr max
  *((volatile uint32_t *) 0x5410004c) = 0x00024000; // Post processing register

  // Layer 1 quadrant 0
  *((volatile uint32_t *) 0x51100104) = 0x0042803e; // Rows
  *((volatile uint32_t *) 0x51100108) = 0x0002803e; // Columns
  *((volatile uint32_t *) 0x51100110) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x51100114) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x51100118) = 0x00000021; // Stride
  *((volatile uint32_t *) 0x5110011c) = 0x00008800; // SRAM write ptr
  *((volatile uint32_t *) 0x51100124) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x51100130) = 0x0088cba0; // Layer control
  *((volatile uint32_t *) 0x51100134) = 0x000e8000; // Layer control 2
  *((volatile uint32_t *) 0x51100138) = 0x000000e8; // Mask count
  *((volatile uint32_t *) 0x51100140) = 0x0000001d; // Output channel count
  *((volatile uint32_t *) 0x51100144) = 0x0000001f; // TRAM ptr max
  *((volatile uint32_t *) 0x5110014c) = 0x00024000; // Post processing register

  // Layer 1 quadrant 1
  *((volatile uint32_t *) 0x52100104) = 0x0042803e; // Rows
  *((volatile uint32_t *) 0x52100108) = 0x0002803e; // Columns
  *((volatile uint32_t *) 0x52100110) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x52100114) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x52100118) = 0x00000021; // Stride
  *((volatile uint32_t *) 0x5210011c) = 0x00008800; // SRAM write ptr
  *((volatile uint32_t *) 0x52100124) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x52100130) = 0x00880ba0; // Layer control
  *((volatile uint32_t *) 0x52100134) = 0x000e8000; // Layer control 2
  *((volatile uint32_t *) 0x52100138) = 0x000000e8; // Mask count
  *((volatile uint32_t *) 0x52100140) = 0x0000001d; // Output channel count
  *((volatile uint32_t *) 0x52100144) = 0x0000001f; // TRAM ptr max
  *((volatile uint32_t *) 0x5210014c) = 0x00024000; // Post processing register

  // Layer 1 quadrant 2
  *((volatile uint32_t *) 0x53100104) = 0x0042803e; // Rows
  *((volatile uint32_t *) 0x53100108) = 0x0002803e; // Columns
  *((volatile uint32_t *) 0x53100110) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x53100114) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x53100118) = 0x00000021; // Stride
  *((volatile uint32_t *) 0x5310011c) = 0x00008800; // SRAM write ptr
  *((volatile uint32_t *) 0x53100124) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x53100130) = 0x00880ba0; // Layer control
  *((volatile uint32_t *) 0x53100134) = 0x000e8000; // Layer control 2
  *((volatile uint32_t *) 0x53100138) = 0x000000e8; // Mask count
  *((volatile uint32_t *) 0x53100140) = 0x0000001d; // Output channel count
  *((volatile uint32_t *) 0x53100144) = 0x0000001f; // TRAM ptr max
  *((volatile uint32_t *) 0x5310014c) = 0x00024000; // Post processing register
  *((volatile uint32_t *) 0x53100148) = 0xfff0fff0; // Mask and processor enables

  // Layer 1 quadrant 3
  *((volatile uint32_t *) 0x54100104) = 0x0042803e; // Rows
  *((volatile uint32_t *) 0x54100108) = 0x0002803e; // Columns
  *((volatile uint32_t *) 0x54100110) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x54100114) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x54100118) = 0x00000021; // Stride
  *((volatile uint32_t *) 0x5410011c) = 0x00008800; // SRAM write ptr
  *((volatile uint32_t *) 0x54100124) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x54100130) = 0x00880ba0; // Layer control
  *((volatile uint32_t *) 0x54100134) = 0x000e8000; // Layer control 2
  *((volatile uint32_t *) 0x54100138) = 0x000000e8; // Mask count
  *((volatile uint32_t *) 0x54100140) = 0x0000001d; // Output channel count
  *((volatile uint32_t *) 0x54100144) = 0x0000001f; // TRAM ptr max
  *((volatile uint32_t *) 0x5410014c) = 0x00024000; // Post processing register
  *((volatile uint32_t *) 0x54100148) = 0x00070007; // Mask and processor enables

  // Layer 2 quadrant 0
  *((volatile uint32_t *) 0x51100204) = 0x0022801e; // Rows
  *((volatile uint32_t *) 0x51100208) = 0x0002801e; // Columns
  *((volatile uint32_t *) 0x51100210) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x51100214) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x51100218) = 0x00000021; // Stride
  *((volatile uint32_t *) 0x5110021c) = 0x00008000; // SRAM write ptr
  *((volatile uint32_t *) 0x51100224) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x5110022c) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x51100230) = 0x00886ba0; // Layer control
  *((volatile uint32_t *) 0x51100234) = 0x001d8000; // Layer control 2
  *((volatile uint32_t *) 0x51100238) = 0x000001d8; // Mask count
  *((volatile uint32_t *) 0x51100240) = 0x0000003b; // Output channel count
  *((volatile uint32_t *) 0x51100244) = 0x0000000f; // TRAM ptr max
  *((volatile uint32_t *) 0x5110024c) = 0x00024000; // Post processing register
  *((volatile uint32_t *) 0x51100248) = 0xfff0fff0; // Mask and processor enables

  // Layer 2 quadrant 1
  *((volatile uint32_t *) 0x52100204) = 0x0022801e; // Rows
  *((volatile uint32_t *) 0x52100208) = 0x0002801e; // Columns
  *((volatile uint32_t *) 0x52100210) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x52100214) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x52100218) = 0x00000021; // Stride
  *((volatile uint32_t *) 0x5210021c) = 0x00008000; // SRAM write ptr
  *((volatile uint32_t *) 0x52100224) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x5210022c) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x52100230) = 0x00880ba0; // Layer control
  *((volatile uint32_t *) 0x52100234) = 0x001d8000; // Layer control 2
  *((volatile uint32_t *) 0x52100238) = 0x000001d8; // Mask count
  *((volatile uint32_t *) 0x52100240) = 0x0000003b; // Output channel count
  *((volatile uint32_t *) 0x52100244) = 0x0000000f; // TRAM ptr max
  *((volatile uint32_t *) 0x5210024c) = 0x00024000; // Post processing register
  *((volatile uint32_t *) 0x52100248) = 0xffffffff; // Mask and processor enables

  // Layer 2 quadrant 2
  *((volatile uint32_t *) 0x53100204) = 0x0022801e; // Rows
  *((volatile uint32_t *) 0x53100208) = 0x0002801e; // Columns
  *((volatile uint32_t *) 0x53100210) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x53100214) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x53100218) = 0x00000021; // Stride
  *((volatile uint32_t *) 0x5310021c) = 0x00008000; // SRAM write ptr
  *((volatile uint32_t *) 0x53100224) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x5310022c) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x53100230) = 0x00880ba0; // Layer control
  *((volatile uint32_t *) 0x53100234) = 0x001d8000; // Layer control 2
  *((volatile uint32_t *) 0x53100238) = 0x000001d8; // Mask count
  *((volatile uint32_t *) 0x53100240) = 0x0000003b; // Output channel count
  *((volatile uint32_t *) 0x53100244) = 0x0000000f; // TRAM ptr max
  *((volatile uint32_t *) 0x5310024c) = 0x00024000; // Post processing register
  *((volatile uint32_t *) 0x53100248) = 0x00030003; // Mask and processor enables

  // Layer 2 quadrant 3
  *((volatile uint32_t *) 0x54100204) = 0x0022801e; // Rows
  *((volatile uint32_t *) 0x54100208) = 0x0002801e; // Columns
  *((volatile uint32_t *) 0x54100210) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x54100214) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x54100218) = 0x00000021; // Stride
  *((volatile uint32_t *) 0x5410021c) = 0x00008000; // SRAM write ptr
  *((volatile uint32_t *) 0x54100224) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x5410022c) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x54100230) = 0x00880ba0; // Layer control
  *((volatile uint32_t *) 0x54100234) = 0x001d8000; // Layer control 2
  *((volatile uint32_t *) 0x54100238) = 0x000001d8; // Mask count
  *((volatile uint32_t *) 0x54100240) = 0x0000003b; // Output channel count
  *((volatile uint32_t *) 0x54100244) = 0x0000000f; // TRAM ptr max
  *((volatile uint32_t *) 0x5410024c) = 0x00024000; // Post processing register

  // Layer 3 quadrant 0
  *((volatile uint32_t *) 0x51100304) = 0x0012800e; // Rows
  *((volatile uint32_t *) 0x51100308) = 0x0002800e; // Columns
  *((volatile uint32_t *) 0x51100310) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x51100314) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x51100318) = 0x00000021; // Stride
  *((volatile uint32_t *) 0x5110031c) = 0x00008800; // SRAM write ptr
  *((volatile uint32_t *) 0x51100324) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x51100330) = 0x0088eba0; // Layer control
  *((volatile uint32_t *) 0x51100334) = 0x000e8000; // Layer control 2
  *((volatile uint32_t *) 0x51100338) = 0x000002c8; // Mask count
  *((volatile uint32_t *) 0x5110033c) = 0x000001e0; // Mask offset
  *((volatile uint32_t *) 0x51100340) = 0x0000001d; // Output channel count
  *((volatile uint32_t *) 0x51100344) = 0x00000007; // TRAM ptr max
  *((volatile uint32_t *) 0x5110034c) = 0x00024000; // Post processing register
  *((volatile uint32_t *) 0x51100348) = 0xfff0fff0; // Mask and processor enables

  // Layer 3 quadrant 1
  *((volatile uint32_t *) 0x52100304) = 0x0012800e; // Rows
  *((volatile uint32_t *) 0x52100308) = 0x0002800e; // Columns
  *((volatile uint32_t *) 0x52100310) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x52100314) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x52100318) = 0x00000021; // Stride
  *((volatile uint32_t *) 0x5210031c) = 0x00008800; // SRAM write ptr
  *((volatile uint32_t *) 0x52100324) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x52100330) = 0x00880ba0; // Layer control
  *((volatile uint32_t *) 0x52100334) = 0x000e8000; // Layer control 2
  *((volatile uint32_t *) 0x52100338) = 0x000002c8; // Mask count
  *((volatile uint32_t *) 0x5210033c) = 0x000001e0; // Mask offset
  *((volatile uint32_t *) 0x52100340) = 0x0000001d; // Output channel count
  *((volatile uint32_t *) 0x52100344) = 0x00000007; // TRAM ptr max
  *((volatile uint32_t *) 0x5210034c) = 0x00024000; // Post processing register
  *((volatile uint32_t *) 0x52100348) = 0xffffffff; // Mask and processor enables

  // Layer 3 quadrant 2
  *((volatile uint32_t *) 0x53100304) = 0x0012800e; // Rows
  *((volatile uint32_t *) 0x53100308) = 0x0002800e; // Columns
  *((volatile uint32_t *) 0x53100310) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x53100314) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x53100318) = 0x00000021; // Stride
  *((volatile uint32_t *) 0x5310031c) = 0x00008800; // SRAM write ptr
  *((volatile uint32_t *) 0x53100324) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x53100330) = 0x00880ba0; // Layer control
  *((volatile uint32_t *) 0x53100334) = 0x000e8000; // Layer control 2
  *((volatile uint32_t *) 0x53100338) = 0x000002c8; // Mask count
  *((volatile uint32_t *) 0x5310033c) = 0x000001e0; // Mask offset
  *((volatile uint32_t *) 0x53100340) = 0x0000001d; // Output channel count
  *((volatile uint32_t *) 0x53100344) = 0x00000007; // TRAM ptr max
  *((volatile uint32_t *) 0x5310034c) = 0x00024000; // Post processing register
  *((volatile uint32_t *) 0x53100348) = 0xffffffff; // Mask and processor enables

  // Layer 3 quadrant 3
  *((volatile uint32_t *) 0x54100304) = 0x0012800e; // Rows
  *((volatile uint32_t *) 0x54100308) = 0x0002800e; // Columns
  *((volatile uint32_t *) 0x54100310) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x54100314) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x54100318) = 0x00000021; // Stride
  *((volatile uint32_t *) 0x5410031c) = 0x00008800; // SRAM write ptr
  *((volatile uint32_t *) 0x54100324) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x54100330) = 0x00880ba0; // Layer control
  *((volatile uint32_t *) 0x54100334) = 0x000e8000; // Layer control 2
  *((volatile uint32_t *) 0x54100338) = 0x000002c8; // Mask count
  *((volatile uint32_t *) 0x5410033c) = 0x000001e0; // Mask offset
  *((volatile uint32_t *) 0x54100340) = 0x0000001d; // Output channel count
  *((volatile uint32_t *) 0x54100344) = 0x00000007; // TRAM ptr max
  *((volatile uint32_t *) 0x5410034c) = 0x00024000; // Post processing register
  *((volatile uint32_t *) 0x54100348) = 0xffffffff; // Mask and processor enables

  // Layer 4 quadrant 0
  *((volatile uint32_t *) 0x51100404) = 0x000a8006; // Rows
  *((volatile uint32_t *) 0x51100408) = 0x00028006; // Columns
  *((volatile uint32_t *) 0x51100410) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x51100414) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x51100418) = 0x00000021; // Stride
  *((volatile uint32_t *) 0x5110041c) = 0x00040000; // SRAM write ptr
  *((volatile uint32_t *) 0x51100424) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x5110042c) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x51100430) = 0x00886ba0; // Layer control
  *((volatile uint32_t *) 0x51100434) = 0x000f8000; // Layer control 2
  *((volatile uint32_t *) 0x51100438) = 0x000003d8; // Mask count
  *((volatile uint32_t *) 0x5110043c) = 0x000002e0; // Mask offset
  *((volatile uint32_t *) 0x51100440) = 0x0000001f; // Output channel count
  *((volatile uint32_t *) 0x51100444) = 0x00000003; // TRAM ptr max
  *((volatile uint32_t *) 0x5110044c) = 0x00024000; // Post processing register
  *((volatile uint32_t *) 0x51100448) = 0xfff0fff0; // Mask and processor enables

  // Layer 4 quadrant 1
  *((volatile uint32_t *) 0x52100404) = 0x000a8006; // Rows
  *((volatile uint32_t *) 0x52100408) = 0x00028006; // Columns
  *((volatile uint32_t *) 0x52100410) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x52100414) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x52100418) = 0x00000021; // Stride
  *((volatile uint32_t *) 0x5210041c) = 0x00040000; // SRAM write ptr
  *((volatile uint32_t *) 0x52100424) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x5210042c) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x52100430) = 0x00880ba0; // Layer control
  *((volatile uint32_t *) 0x52100434) = 0x000f8000; // Layer control 2
  *((volatile uint32_t *) 0x52100438) = 0x000003d8; // Mask count
  *((volatile uint32_t *) 0x5210043c) = 0x000002e0; // Mask offset
  *((volatile uint32_t *) 0x52100440) = 0x0000001f; // Output channel count
  *((volatile uint32_t *) 0x52100444) = 0x00000003; // TRAM ptr max
  *((volatile uint32_t *) 0x5210044c) = 0x00024000; // Post processing register
  *((volatile uint32_t *) 0x52100448) = 0xffffffff; // Mask and processor enables

  // Layer 4 quadrant 2
  *((volatile uint32_t *) 0x53100404) = 0x000a8006; // Rows
  *((volatile uint32_t *) 0x53100408) = 0x00028006; // Columns
  *((volatile uint32_t *) 0x53100410) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x53100414) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x53100418) = 0x00000021; // Stride
  *((volatile uint32_t *) 0x5310041c) = 0x00040000; // SRAM write ptr
  *((volatile uint32_t *) 0x53100424) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x5310042c) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x53100430) = 0x00880ba0; // Layer control
  *((volatile uint32_t *) 0x53100434) = 0x000f8000; // Layer control 2
  *((volatile uint32_t *) 0x53100438) = 0x000003d8; // Mask count
  *((volatile uint32_t *) 0x5310043c) = 0x000002e0; // Mask offset
  *((volatile uint32_t *) 0x53100440) = 0x0000001f; // Output channel count
  *((volatile uint32_t *) 0x53100444) = 0x00000003; // TRAM ptr max
  *((volatile uint32_t *) 0x5310044c) = 0x00024000; // Post processing register
  *((volatile uint32_t *) 0x53100448) = 0x00030003; // Mask and processor enables

  // Layer 4 quadrant 3
  *((volatile uint32_t *) 0x54100404) = 0x000a8006; // Rows
  *((volatile uint32_t *) 0x54100408) = 0x00028006; // Columns
  *((volatile uint32_t *) 0x54100410) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x54100414) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x54100418) = 0x00000021; // Stride
  *((volatile uint32_t *) 0x5410041c) = 0x00040000; // SRAM write ptr
  *((volatile uint32_t *) 0x54100424) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x5410042c) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x54100430) = 0x00880ba0; // Layer control
  *((volatile uint32_t *) 0x54100434) = 0x000f8000; // Layer control 2
  *((volatile uint32_t *) 0x54100438) = 0x000003d8; // Mask count
  *((volatile uint32_t *) 0x5410043c) = 0x000002e0; // Mask offset
  *((volatile uint32_t *) 0x54100440) = 0x0000001f; // Output channel count
  *((volatile uint32_t *) 0x54100444) = 0x00000003; // TRAM ptr max
  *((volatile uint32_t *) 0x5410044c) = 0x00024000; // Post processing register

  // Layer 5 quadrant 0
  *((volatile uint32_t *) 0x51100504) = 0x00018003; // Rows
  *((volatile uint32_t *) 0x51100508) = 0x00018003; // Columns
  *((volatile uint32_t *) 0x51100518) = 0x00000010; // Stride
  *((volatile uint32_t *) 0x5110051c) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x51100524) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x51100530) = 0x0088cb20; // Layer control
  *((volatile uint32_t *) 0x51100534) = 0x000e8000; // Layer control 2
  *((volatile uint32_t *) 0x51100538) = 0x000003c8; // Mask count
  *((volatile uint32_t *) 0x5110053c) = 0x000002e0; // Mask offset
  *((volatile uint32_t *) 0x51100540) = 0x0000001d; // Output channel count
  *((volatile uint32_t *) 0x51100544) = 0x00000003; // TRAM ptr max
  *((volatile uint32_t *) 0x5110054c) = 0x00024000; // Post processing register

  // Layer 5 quadrant 1
  *((volatile uint32_t *) 0x52100504) = 0x00018003; // Rows
  *((volatile uint32_t *) 0x52100508) = 0x00018003; // Columns
  *((volatile uint32_t *) 0x52100518) = 0x00000010; // Stride
  *((volatile uint32_t *) 0x5210051c) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x52100524) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x52100530) = 0x00880b20; // Layer control
  *((volatile uint32_t *) 0x52100534) = 0x000e8000; // Layer control 2
  *((volatile uint32_t *) 0x52100538) = 0x000003c8; // Mask count
  *((volatile uint32_t *) 0x5210053c) = 0x000002e0; // Mask offset
  *((volatile uint32_t *) 0x52100540) = 0x0000001d; // Output channel count
  *((volatile uint32_t *) 0x52100544) = 0x00000003; // TRAM ptr max
  *((volatile uint32_t *) 0x5210054c) = 0x00024000; // Post processing register

  // Layer 5 quadrant 2
  *((volatile uint32_t *) 0x53100504) = 0x00018003; // Rows
  *((volatile uint32_t *) 0x53100508) = 0x00018003; // Columns
  *((volatile uint32_t *) 0x53100518) = 0x00000010; // Stride
  *((volatile uint32_t *) 0x5310051c) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x53100524) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x53100530) = 0x00880b20; // Layer control
  *((volatile uint32_t *) 0x53100534) = 0x000e8000; // Layer control 2
  *((volatile uint32_t *) 0x53100538) = 0x000003c8; // Mask count
  *((volatile uint32_t *) 0x5310053c) = 0x000002e0; // Mask offset
  *((volatile uint32_t *) 0x53100540) = 0x0000001d; // Output channel count
  *((volatile uint32_t *) 0x53100544) = 0x00000003; // TRAM ptr max
  *((volatile uint32_t *) 0x5310054c) = 0x00024000; // Post processing register
  *((volatile uint32_t *) 0x53100548) = 0xfffcfffc; // Mask and processor enables

  // Layer 5 quadrant 3
  *((volatile uint32_t *) 0x54100504) = 0x00018003; // Rows
  *((volatile uint32_t *) 0x54100508) = 0x00018003; // Columns
  *((volatile uint32_t *) 0x54100518) = 0x00000010; // Stride
  *((volatile uint32_t *) 0x5410051c) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x54100524) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x54100530) = 0x00880b20; // Layer control
  *((volatile uint32_t *) 0x54100534) = 0x000e8000; // Layer control 2
  *((volatile uint32_t *) 0x54100538) = 0x000003c8; // Mask count
  *((volatile uint32_t *) 0x5410053c) = 0x000002e0; // Mask offset
  *((volatile uint32_t *) 0x54100540) = 0x0000001d; // Output channel count
  *((volatile uint32_t *) 0x54100544) = 0x00000003; // TRAM ptr max
  *((volatile uint32_t *) 0x5410054c) = 0x00024000; // Post processing register
  *((volatile uint32_t *) 0x54100548) = 0xffffffff; // Mask and processor enables

  // Layer 6 quadrant 0
  *((volatile uint32_t *) 0x51100604) = 0x00010000; // Rows
  *((volatile uint32_t *) 0x51100608) = 0x00010000; // Columns
  *((volatile uint32_t *) 0x51100618) = 0x00000010; // Stride
  *((volatile uint32_t *) 0x5110061c) = 0x00000400; // SRAM write ptr
  *((volatile uint32_t *) 0x51100620) = 0x00000001; // Write ptr time slot offs
  *((volatile uint32_t *) 0x51100624) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x5110062c) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x51100630) = 0x00012920; // Layer control
  *((volatile uint32_t *) 0x51100634) = 0x0000800f; // Layer control 2
  *((volatile uint32_t *) 0x51100638) = 0x000023d8; // Mask count
  *((volatile uint32_t *) 0x5110063c) = 0x000022e0; // Mask offset
  *((volatile uint32_t *) 0x51100640) = 0x0000001f; // Output channel count
  *((volatile uint32_t *) 0x5110060c) = 0x00000100; // 1D
  *((volatile uint32_t *) 0x5110064c) = 0x00027000; // Post processing register
  *((volatile uint32_t *) 0x51100648) = 0xffffffff; // Mask and processor enables

  // Layer 6 quadrant 1
  *((volatile uint32_t *) 0x52100604) = 0x00010000; // Rows
  *((volatile uint32_t *) 0x52100608) = 0x00010000; // Columns
  *((volatile uint32_t *) 0x52100618) = 0x00000010; // Stride
  *((volatile uint32_t *) 0x5210061c) = 0x00000400; // SRAM write ptr
  *((volatile uint32_t *) 0x52100620) = 0x00000001; // Write ptr time slot offs
  *((volatile uint32_t *) 0x52100624) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x5210062c) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x52100630) = 0x00010920; // Layer control
  *((volatile uint32_t *) 0x52100634) = 0x0000800f; // Layer control 2
  *((volatile uint32_t *) 0x52100638) = 0x000023d8; // Mask count
  *((volatile uint32_t *) 0x5210063c) = 0x000022e0; // Mask offset
  *((volatile uint32_t *) 0x52100640) = 0x0000001f; // Output channel count
  *((volatile uint32_t *) 0x5210060c) = 0x00000100; // 1D
  *((volatile uint32_t *) 0x5210064c) = 0x00026000; // Post processing register
  *((volatile uint32_t *) 0x52100648) = 0x3fff3fff; // Mask and processor enables

  // Layer 6 quadrant 2
  *((volatile uint32_t *) 0x53100604) = 0x00010000; // Rows
  *((volatile uint32_t *) 0x53100608) = 0x00010000; // Columns
  *((volatile uint32_t *) 0x53100618) = 0x00000010; // Stride
  *((volatile uint32_t *) 0x5310061c) = 0x00000400; // SRAM write ptr
  *((volatile uint32_t *) 0x53100620) = 0x00000001; // Write ptr time slot offs
  *((volatile uint32_t *) 0x53100624) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x5310062c) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x53100630) = 0x00010920; // Layer control
  *((volatile uint32_t *) 0x53100634) = 0x0000800f; // Layer control 2
  *((volatile uint32_t *) 0x53100638) = 0x000023d8; // Mask count
  *((volatile uint32_t *) 0x5310063c) = 0x000022e0; // Mask offset
  *((volatile uint32_t *) 0x53100640) = 0x0000001f; // Output channel count
  *((volatile uint32_t *) 0x5310060c) = 0x00000100; // 1D
  *((volatile uint32_t *) 0x5310064c) = 0x00026000; // Post processing register

  // Layer 6 quadrant 3
  *((volatile uint32_t *) 0x54100604) = 0x00010000; // Rows
  *((volatile uint32_t *) 0x54100608) = 0x00010000; // Columns
  *((volatile uint32_t *) 0x54100618) = 0x00000010; // Stride
  *((volatile uint32_t *) 0x5410061c) = 0x00000400; // SRAM write ptr
  *((volatile uint32_t *) 0x54100620) = 0x00000001; // Write ptr time slot offs
  *((volatile uint32_t *) 0x54100624) = 0x00008000; // Write ptr mask offs
  *((volatile uint32_t *) 0x5410062c) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x54100630) = 0x00010920; // Layer control
  *((volatile uint32_t *) 0x54100634) = 0x0000800f; // Layer control 2
  *((volatile uint32_t *) 0x54100638) = 0x000023d8; // Mask count
  *((volatile uint32_t *) 0x5410063c) = 0x000022e0; // Mask offset
  *((volatile uint32_t *) 0x54100640) = 0x0000001f; // Output channel count
  *((volatile uint32_t *) 0x5410060c) = 0x00000100; // 1D
  *((volatile uint32_t *) 0x5410064c) = 0x00026000; // Post processing register


  return CNN_OK;
}

int cnn_start(void)
{
  cnn_time = 0;

  *((volatile uint32_t *) 0x51000000) = 0x0010080e; // Enable quadrant 0
  *((volatile uint32_t *) 0x52000000) = 0x0010080f; // Enable quadrant 1
  *((volatile uint32_t *) 0x53000000) = 0x0010080f; // Enable quadrant 2
  *((volatile uint32_t *) 0x54000000) = 0x0010080f; // Enable quadrant 3

#ifdef CNN_INFERENCE_TIMER
  MXC_TMR_SW_Start(CNN_INFERENCE_TIMER);
#endif

  CNN_START; // Allow capture of processing time
  *((volatile uint32_t *) 0x51000000) = 0x0010000f; // Master enable quadrant 0

  return CNN_OK;
}

// Custom unload for this network: 32-bit data, shape: [2, 1, 1]
int cnn_unload(uint32_t *out_buf)
{
  volatile uint32_t *addr;

  addr = (volatile uint32_t *) 0x51801000;
  *out_buf++ = *addr++;
  *out_buf++ = *addr++;

  return CNN_OK;
}

int cnn_enable(uint32_t clock_source, uint32_t clock_divider)
{
  // Reset all domains, restore power to CNN
  MXC_GCFR->reg3 = 0xf; // Reset
  MXC_GCFR->reg1 = 0xf; // Mask memory
  MXC_GCFR->reg0 = 0xf; // Power
  MXC_Delay(MSEC(10)); // Wait for load switches
  MXC_GCFR->reg2 = 0x0; // Iso
  MXC_GCFR->reg3 = 0x0; // Reset

  if (clock_source == MXC_S_GCR_PCLKDIV_CNNCLKSEL_IPLL)
    while ((MXC_GCR->ipll_ctrl & MXC_F_GCR_IPLL_CTRL_RDY) != MXC_F_GCR_IPLL_CTRL_RDY) ; // Wait for PLL

  MXC_GCR->pclkdiv = (MXC_GCR->pclkdiv & ~(MXC_F_GCR_PCLKDIV_CNNCLKDIV | MXC_F_GCR_PCLKDIV_CNNCLKSEL))
                     | clock_divider | clock_source;
  MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CNN); // Enable CNN clock

  NVIC_SetVector(CNN_IRQn, CNN_ISR); // Set CNN complete vector

  return CNN_OK;
}

int cnn_boost_enable(mxc_gpio_regs_t *port, uint32_t pin)
{
  mxc_gpio_cfg_t gpio_out;
  gpio_out.port = port;
  gpio_out.mask = pin;
  gpio_out.pad = MXC_GPIO_PAD_NONE;
  gpio_out.func = MXC_GPIO_FUNC_OUT;
  MXC_GPIO_Config(&gpio_out);
  MXC_GPIO_OutSet(gpio_out.port, gpio_out.mask);

  return CNN_OK;
}

int cnn_boost_disable(mxc_gpio_regs_t *port, uint32_t pin)
{
  mxc_gpio_cfg_t gpio_out;
  gpio_out.port = port;
  gpio_out.mask = pin;
  gpio_out.pad = MXC_GPIO_PAD_NONE;
  gpio_out.func = MXC_GPIO_FUNC_OUT;
  MXC_GPIO_Config(&gpio_out);
  MXC_GPIO_OutClr(gpio_out.port, gpio_out.mask);

  return CNN_OK;
}

int cnn_disable(void)
{
  // Disable CNN clock
  MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_CNN);

  // Disable power to CNN
  MXC_GCFR->reg3 = 0xf; // Reset
  MXC_GCFR->reg2 = 0xf; // Iso
  MXC_GCFR->reg0 = 0x0; // Power
  MXC_GCFR->reg1 = 0x0; // Mask memory
  MXC_GCFR->reg3 = 0x0; // Reset

  return CNN_OK;
}

