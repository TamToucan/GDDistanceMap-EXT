#ifndef _MATHSTUFF_GRID_2D_H
#define _MATHSTUFF_GRID_2D_H

namespace MathStuff {

template<typename Element>
class Grid2D {
	std::vector<Element> mGrid;
	int mWidth;
	int mHeight;

public:
	Grid2D() : mWidth(0), mHeight(0) {}

	Grid2D(int width, int height) {
		init(width, height);
	}

	Grid2D(int width, int height, const Element& val) {
		init(width, height, val);
	}

	int width() const { return mWidth; }
	int height() const { return mHeight; }

	void init(int w, int h)
	{
		mWidth = w;
		mHeight = h;
		mGrid.resize(mWidth * mHeight);
	}
	void init(int w, int h, const Element val)
	{
		init(w, h);
		std::fill(mGrid.begin(), mGrid.end(), val);
	}

	bool in(int x, int y) const
	{
		return x >= 0 && x < mWidth && y >= 0 && y < mHeight;
	}

	int indexFor(int x, int y) const
	{
		return y * mWidth + x;
	}

	// Get Const
	const Element& get(int x, int y) const
	{
		return get(indexFor(x, y));
	}
	const Element& get(int idx) const
	{
		return mGrid[idx];
	}

	// Put
	void put(int x, int y, const Element& val)
	{
		return put(indexFor(x, y), val);
	}
	void put(int idx, const Element& val)
	{
		mGrid[idx] = val;
	}

	// Non-cost 
	Element& at(int x, int y)
	{
		return at(indexFor(x, y));
	}
	Element& at(int idx)
	{
		return mGrid[idx];
	}
};

}

#endif

