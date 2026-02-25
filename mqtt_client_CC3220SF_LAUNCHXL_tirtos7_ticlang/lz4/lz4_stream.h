/* lz4_stream.h

Copyright (c) 2011-2016, Yann Collet. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#ifndef LZ4_STREAM_H_
#define LZ4_STREAM_H_

//******************************************************************************
//
//! \addtogroup lz4_api
//! @{
//
//******************************************************************************

#include "lz4.h"

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

//******************************************************************************
//
//! \brief LZ4 streaming API state.
//
//******************************************************************************
typedef enum {
    //! Next byte is the token
    LZ4_BLOCK_SIZE,
    //! Next byte is the token
    LZ4_TOKEN,
    //! Next byte is the literal length
    LZ4_LITERAL_LENGTH,
    //! Next byte is a literal character
    LZ4_LITERAL,
    //! Next byte is the match length
    LZ4_MATCH_LENGTH,
    //! Next byte is the low match address offset
    LZ4_MATCH_OFFSET_LOW,
    //! Next byte is the high match address offset
    LZ4_MATCH_OFFSET_HIGH
} LZ4_streamState;

//******************************************************************************
//
//! \brief Decompression state for a streaming LZ4 block.
//
//******************************************************************************
typedef struct LZ4_streamDecompressBlockState {
    //! Decompress stream state.
    LZ4_streamState state;
    //! Length of literals.
    uint16_t literalLength;
    //! Length of match.
    uint16_t matchLength;
    //! Offset address of match.
    uint16_t matchOffset;
    //! Pointer to the destination data buffer to write.
    uint8_t *dstPtr;
    //! Destination buffer pointer origin.
    uint8_t *dstOrigin;
    //! Length of the destination data buffer.
    //!
    uint32_t dstLength;
} LZ4_streamDecompressBlockState;

//******************************************************************************
//
//! \brief Decompression parameters for a streaming LZ4 block.
//
//******************************************************************************
typedef struct LZ4_streamDecompressBlockParams {
    //! Pointer to the destination data buffer to store uncompressed data.
    void *dst;
    //! Length of the destination data buffer.
    int32_t dstLength;
    //! Compressed LZ4 data contains block size.
    //! - false
    //! - true
    bool containsBlockSize;
} LZ4_streamDecompressBlockParams;

//******************************************************************************
//
//! \brief Initialize LZ4 stream decompression.
//!
//! Initialize LZ4 decompression using a stream of data blocks. The streaming
//! API's can be used when data is sent in chunks such as over-the-air or wired
//! serial communication and removes the need to buffer then entire compressed
//! data before running decompression, reducing total system memory used.
//!
//! This function must be first called to initialize the state before calling
//! LZ4_streamDecompressBlock() to decompress data.
//!
//! \param params Pointer to the LZ4 decompression strean parameter structure.
//! \param state  Pointer to the LZ4 decompression stream state.
//! \param status Pointer to a LZ4 status that will contain the result status.
//! \return none
//
//******************************************************************************
extern void LZ4_streamDecompressBlockInit(const LZ4_streamDecompressBlockParams *params, LZ4_streamDecompressBlockState *state, LZ4_status *status);

//******************************************************************************
//
//! \brief Decompress a single LZ4 block as a stream of data.
//!
//! Continue decompression using a stream of data blocks. The streaming API's
//! can be used when data is sent in chunks such as over-the-air or wired
//! serial communication and removes the need to buffer then entire compressed
//! data before running decompression, reducing total system memory used.
//!
//! The LZ4_streamDecompressBlockInit() function must first be called to
//! initialize the state before decompressing data.
//!
//! \param state    Pointer to the LZ4 decompression stream state.
//! \param data     Pointer to a block of data to continue decompression with.
//! \param length   Length of data block in bytes.
//! \param status   Pointer to a LZ4 status that will contain the result status.
//! \return The current length of decompressed data.
//
//******************************************************************************
extern uint32_t LZ4_streamDecompressBlock(LZ4_streamDecompressBlockState *state, const void *data, uint16_t length, LZ4_status *status);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

//******************************************************************************
//
// Close the Doxygen group.
//! @}
//
//******************************************************************************

#endif /* LZ4_STREAM_H_ */
