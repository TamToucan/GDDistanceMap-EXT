#ifndef STUFF_TGA_H
#define STUFF_TGA_H

namespace Stuff {

class TGA
{
    // TGA ImageType values
    enum {
        TGA_BGRA=2,
        TGA_GREY=3,
        TGA_BGRA_RLE=10,
        TGA_GRAY_RLE=11
    };
    struct TGAHeader {
        unsigned char m_idLen;
        unsigned char m_colorMapType;
        unsigned char m_imageType;
        unsigned char m_colorMapIndexLSB, m_colorMapIndexMSB;
        unsigned char m_colorMapLengthLSB, m_colorMapLengthMSB;
        unsigned char m_colorMapSize;
        unsigned char m_xOriginLSB, m_xOriginMSB;
        unsigned char m_yOriginLSB, m_yOriginMSB;
        unsigned char m_widthLSB, m_widthMSB;
        unsigned char m_heightLSB, m_heightMSB;
        unsigned char m_pixelDepth;
        unsigned char m_attrBits;
    };

public:
    static bool loadFromFile(const char* pFilename,
                             unsigned int* pWidth,
                             unsigned int* pHeight,
                             unsigned int* pPixelDepth,
                             unsigned char** ppPixelData);

    static bool saveToFile(const char* pFilename,
                           const unsigned int width,
                           const unsigned int height,
                           const unsigned int pixelDepth,
                           const unsigned char* pPixelData);

    static unsigned char* RGBtoRGBA(const unsigned char* pPixelData,
                                    const unsigned int width,
                                    const unsigned int height);
};

} // namespace

#endif
