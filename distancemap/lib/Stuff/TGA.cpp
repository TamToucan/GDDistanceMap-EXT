#include <fstream>
#include <cstring>

#include "TGA.hpp"

#include "Debug.h"

namespace Stuff {

static unsigned char* swapPixelData(const unsigned char* pPixelData,
                           const unsigned int width,
                           const unsigned int height,
                           const unsigned int pixelDepth)
{
    // Copy the data swapping RGB(A) to BGR(A)
    // (or vice versa) if needed
    const unsigned int l_bytesPerPixel = pixelDepth/8;
    const unsigned long l_imageSize = width * height * l_bytesPerPixel;
    unsigned char* l_pNewPixels = new unsigned char[ l_imageSize ];
    unsigned char* l_pCurPixel = l_pNewPixels;

    for (unsigned int i=0; i < width * height; ++i)
    {
        // Hacky way to copy the pixels swapping if needed
        // Defines the src->dest mapping of the (up to 4) bytes per pixel
        // So greyscale only uses the first value (0)
        // RGB uses the first 3 and maps 2->0 (B->r), 1->1 (G->g), 0->2(R->b)
        // RGBA uses all 4, as per RGB plus 3->3 (A->a)
        //
        static unsigned int l_copyOffsets[4][4] = {
            { 0, 0, 0, 0 },
            { 0, 0, 0, 0 }, // 16 bit pixel depth not supported
            { 2, 1, 0, 3 },
            { 2, 1, 0, 3 }
        };
        for (unsigned int j=0; j < l_bytesPerPixel; ++j)
        {
            unsigned int l_offset = l_copyOffsets[l_bytesPerPixel-1][j];
            *l_pCurPixel++ = pPixelData[i*l_bytesPerPixel + l_offset];
        }
    }
    return l_pNewPixels;
}

///////////////////////////////////////////////////////////////////////

bool TGA::loadFromFile(const char* pFilename,
                       unsigned int* pWidth,
                       unsigned int* pHeight,
                       unsigned int* pPixelDepth,
                       unsigned char** ppPixelData)
{
    *ppPixelData = 0;
    std::ifstream l_ifs(pFilename, std::ifstream::binary);
    if (! pFilename || ! l_ifs)
    {
        return false;
    }
    // Read the header
    // - This seems insane, can only call get() with CHAR, but then I'll have
    // to be careful of the sign e.g. char c = 0x80 then need to cast to unsigned char
    // before I do anything with it
    // So put horrible casts here since what we what to read are UNSIGNED 8 bit values
    //
    TGAHeader l_tgaHdr;
    l_ifs.get(reinterpret_cast<char&>(l_tgaHdr.m_idLen));
    l_ifs.get(reinterpret_cast<char&>(l_tgaHdr.m_colorMapType));
    l_ifs.get(reinterpret_cast<char&>(l_tgaHdr.m_imageType));
    l_ifs.get(reinterpret_cast<char&>(l_tgaHdr.m_colorMapIndexLSB));
    l_ifs.get(reinterpret_cast<char&>(l_tgaHdr.m_colorMapIndexMSB));
    l_ifs.get(reinterpret_cast<char&>(l_tgaHdr.m_colorMapLengthLSB));
    l_ifs.get(reinterpret_cast<char&>(l_tgaHdr.m_colorMapLengthMSB));
    l_ifs.get(reinterpret_cast<char&>(l_tgaHdr.m_colorMapSize));
    l_ifs.get(reinterpret_cast<char&>(l_tgaHdr.m_xOriginLSB));
    l_ifs.get(reinterpret_cast<char&>(l_tgaHdr.m_xOriginMSB));
    l_ifs.get(reinterpret_cast<char&>(l_tgaHdr.m_yOriginLSB));
    l_ifs.get(reinterpret_cast<char&>(l_tgaHdr.m_yOriginMSB));
    l_ifs.get(reinterpret_cast<char&>(l_tgaHdr.m_widthLSB));
    l_ifs.get(reinterpret_cast<char&>(l_tgaHdr.m_widthMSB));
    l_ifs.get(reinterpret_cast<char&>(l_tgaHdr.m_heightLSB));
    l_ifs.get(reinterpret_cast<char&>(l_tgaHdr.m_heightMSB));
    l_ifs.get(reinterpret_cast<char&>(l_tgaHdr.m_pixelDepth));
    l_ifs.get(reinterpret_cast<char&>(l_tgaHdr.m_attrBits));
    l_ifs.seekg(l_tgaHdr.m_idLen, std::ios::cur);

    if (1 == l_tgaHdr.m_colorMapType)
    {
        return false;
    }
    switch (l_tgaHdr.m_pixelDepth)
    {
    case 8:
    case 24:
    case 32:
        break;
    default:
        {
            return false;
        }
    }
    char l_alphaDepth = (l_tgaHdr.m_attrBits&0xf);
    if ((l_alphaDepth != 0) && (l_alphaDepth != 8))
    {
        return false;
    }

    *pWidth = (l_tgaHdr.m_widthMSB << 8) | (l_tgaHdr.m_widthLSB);
    *pHeight = (l_tgaHdr.m_heightMSB << 8) | (l_tgaHdr.m_heightLSB);
    *pPixelDepth = (l_tgaHdr.m_pixelDepth);

    long l_numPixels = (*pWidth) * (*pHeight);
    const long l_bytesPerPixel = (*pPixelDepth/8);
    const long l_size = l_numPixels * l_bytesPerPixel;
    unsigned char* l_pPixelData = new unsigned char[ l_size ];

    if (   (TGA_BGRA_RLE == l_tgaHdr.m_imageType)
        || (TGA_GRAY_RLE == l_tgaHdr.m_imageType) )
    {
        char* l_pAPixel = new char[l_bytesPerPixel];
        unsigned char* l_pCurPixel = l_pPixelData;
        while (l_numPixels > 0)
        {
            char l_byte;
            l_ifs.read(&l_byte,1);
            int l_rleLen = 1 + (l_byte & 0x7f);
            l_numPixels -= l_rleLen;

            // Top bit set => RLE
            if (l_byte & 0x80)
            {
                l_ifs.read(l_pAPixel, l_bytesPerPixel);
                while (l_rleLen--)
                {
                    for (int i=0; i < l_bytesPerPixel; ++i)
                    {
                        *l_pCurPixel++ = l_pAPixel[i];
                    }
                }
            }
            // RLE len is number of normal pixels
            else
            {
                l_ifs.read(reinterpret_cast<char*>(l_pCurPixel),
                          l_bytesPerPixel*l_rleLen);
                l_pCurPixel += l_bytesPerPixel*l_rleLen;
            }
        }
        delete[] l_pAPixel;
    }
    // Just read all the pixels
    else
    {
        l_ifs.read(reinterpret_cast<char*>(l_pPixelData), l_size);
    }

    l_ifs.close();
    if (!l_ifs)
    {
        return false;
    }

    // Bit5 is screen origin
    // set => top left => swap the rows
    if (l_tgaHdr.m_attrBits&0x20)
    {
        int l_oneRowSize = (*pWidth) * l_bytesPerPixel;
        char* l_pOneRow = new char[l_oneRowSize];
        unsigned char* l_pSrcRow = l_pPixelData;
        unsigned char* l_pDstRow = l_pPixelData + l_size - l_oneRowSize;
        for (unsigned int i=0; i < *pHeight/2; ++i)
        {
            memcpy(l_pOneRow, l_pDstRow, l_oneRowSize);
            memcpy(l_pDstRow, l_pSrcRow, l_oneRowSize);
            memcpy(l_pSrcRow, l_pOneRow, l_oneRowSize);
            l_pDstRow -= l_oneRowSize;
            l_pSrcRow += l_oneRowSize;
        }
        delete[] l_pOneRow;
    }

    // swap BGR(A) to RGB(A)
    // Ok so for greyscale this makes another (temp) copy of the
    // pixels which isn't needed, but who cares
    *ppPixelData = swapPixelData(l_pPixelData, *pWidth, *pHeight, *pPixelDepth);
    delete[] l_pPixelData;
    return true;
}

///////////////////////////////////////////////////////////////////////

bool TGA::saveToFile(const char* pFilename,
                     const unsigned int width,
                     const unsigned int height,
                     const unsigned int pixelDepth,
                     const unsigned char* pPixelData)
{
    std::ofstream l_ofs(pFilename, std::ofstream::binary);
    if (! pFilename || ! l_ofs)
    {
        return false;
    }

    char l_imageType;
    switch (pixelDepth)
    {
    case 8:
        l_imageType = TGA_GREY;
        break;
    case 24:
    case 32:
        l_imageType = TGA_BGRA;
        break;
    default:
        {
            return false;
        }
    }
    unsigned char* l_pNewPixels;
    l_pNewPixels = swapPixelData(pPixelData, width, height, pixelDepth);

    // Create the Header
    // -Macro to put a word in LSB,MSB format
#define LSB_MSB(val) ((val)&0xff), ((val)&0xff00)>>8
    TGAHeader l_tgaHdr = {
        0,                   // ID Len
        0,                   // No Color Map
        l_imageType,
        LSB_MSB(0),          // Color map index
        LSB_MSB(0),          // Color map len
        0,                   // Color map size
        LSB_MSB(0),          // X origin
        LSB_MSB(0),          // Y origin
        LSB_MSB(width),
        LSB_MSB(height),
        pixelDepth,
        8                    // Attr bit (Alpha?)
    };

    // Write it - Note can't write in one go since compiler
    // will probably align on 4 byte boundaries
    l_ofs.put(l_tgaHdr.m_idLen);
    l_ofs.put(l_tgaHdr.m_colorMapType);
    l_ofs.put(l_tgaHdr.m_imageType);
    l_ofs.put(l_tgaHdr.m_colorMapIndexLSB);
    l_ofs.put(l_tgaHdr.m_colorMapIndexMSB);
    l_ofs.put(l_tgaHdr.m_colorMapLengthLSB);
    l_ofs.put(l_tgaHdr.m_colorMapLengthMSB);
    l_ofs.put(l_tgaHdr.m_colorMapSize);
    l_ofs.put(l_tgaHdr.m_xOriginLSB);
    l_ofs.put(l_tgaHdr.m_xOriginMSB);
    l_ofs.put(l_tgaHdr.m_yOriginLSB);
    l_ofs.put(l_tgaHdr.m_yOriginMSB);
    l_ofs.put(l_tgaHdr.m_widthLSB);
    l_ofs.put(l_tgaHdr.m_widthMSB);
    l_ofs.put(l_tgaHdr.m_heightLSB);
    l_ofs.put(l_tgaHdr.m_heightMSB);
    l_ofs.put(l_tgaHdr.m_pixelDepth);
    l_ofs.put(l_tgaHdr.m_attrBits);

    // Write the pixels
    l_ofs.write(reinterpret_cast<char*>(l_pNewPixels),
                (width * height * (pixelDepth/8)));
    delete[] l_pNewPixels;

    l_ofs.close();
    return (! l_ofs) ? false : true;
}

///////////////////////////////////////////////////////////////////////

unsigned char* TGA::RGBtoRGBA(const unsigned char* pPixelData,
                              const unsigned int width,
                              const unsigned int height)
{
    unsigned char* l_pNew = new unsigned char[width*height*4];
    unsigned char* l_pDst = l_pNew;
    for (unsigned int i=0; i < width * height; ++i)
    {
        *l_pDst++ = *pPixelData++;
        *l_pDst++ = *pPixelData++;
        *l_pDst++ = *pPixelData++;
        *l_pDst++ = 0xff;
    }
    return l_pNew;


}
///////////////////////////////////////////////////////////////////////

} // namespace

