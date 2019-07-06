#pragma once
class FeatureTracker
{
public:
	FeatureTracker();

	void readIntrinsicParameter(const string &calib_file);
	void addPoints();
	void updateID();
	void showUndistortion();

	
	virtual ~FeatureTracker();
};