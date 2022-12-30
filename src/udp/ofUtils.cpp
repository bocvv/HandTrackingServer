/*************************************************************************
	> File Name: ofUtils_less.cpp
	> Author: 
	> Mail: 
	> Created Time: 四  8/13 14:08:40 2020
 ************************************************************************/

#include <iostream>
#include <chrono>
#include <numeric>
#include <locale>
#include <cstdarg>
#include "ofUtils.h"
#include "ofFileUtils.h"
#include "ofLog.h"

using namespace std;



namespace{
	bool enableDataPath = true;

    //--------------------------------------------------
    string defaultDataPath(){
    #if defined TARGET_OSX
        try{
            return std::filesystem::canonical(ofFilePath::join(ofFilePath::getCurrentExeDir(),  "../../../data/")).string();
        }catch(...){
            return ofFilePath::join(ofFilePath::getCurrentExeDir(),  "../../../data/");
        }
    #elif defined TARGET_ANDROID
        return string("sdcard/");
    #else
        try{
            return std::filesystem::canonical(ofFilePath::join(ofFilePath::getCurrentExeDir(),  "data/")).make_preferred().string();
        }catch(...){
            return ofFilePath::join(ofFilePath::getCurrentExeDir(),  "data/");
        }
    #endif
    }

    //--------------------------------------------------
    std::filesystem::path & defaultWorkingDirectory(){
            static auto * defaultWorkingDirectory = new std::filesystem::path();
            return * defaultWorkingDirectory;
    }

    //--------------------------------------------------
    std::filesystem::path & dataPathRoot(){
            static auto * dataPathRoot = new std::filesystem::path(defaultDataPath());
            return *dataPathRoot;
    }
}


//--------------------------------------------------
string ofToDataPath(const std::filesystem::path & path, bool makeAbsolute){
    if (makeAbsolute && path.is_absolute())
        return path.string();
    
	if (!enableDataPath)
        return path.string();

    bool hasTrailingSlash = !path.empty() && path.generic_string().back()=='/';

	// if our Current Working Directory has changed (e.g. file open dialog)
#ifdef TARGET_WIN32
	if (defaultWorkingDirectory() != std::filesystem::current_path()) {
		// change our cwd back to where it was on app load
		bool ret = ofRestoreWorkingDirectoryToDefault();
		if(!ret){
			ofLogWarning("ofUtils") << "ofToDataPath: error while trying to change back to default working directory " << defaultWorkingDirectory();
		}
	}
#endif

	// this could be performed here, or wherever we might think we accidentally change the cwd, e.g. after file dialogs on windows
	const auto  & dataPath = dataPathRoot();
	std::filesystem::path inputPath(path);
	std::filesystem::path outputPath;

	// if path is already absolute, just return it
	if (inputPath.is_absolute()) {
		try {
            auto outpath = std::filesystem::canonical(inputPath).make_preferred();
            if(std::filesystem::is_directory(outpath) && hasTrailingSlash){
                return ofFilePath::addTrailingSlash(outpath.string());
            }else{
                return outpath.string();
            }
		}
		catch (...) {
            return inputPath.string();
		}
	}

	// here we check whether path already refers to the data folder by looking for common elements
	// if the path begins with the full contents of dataPathRoot then the data path has already been added
	// we compare inputPath.toString() rather that the input var path to ensure common formatting against dataPath.toString()
    auto dirDataPath = dataPath.string();
	// also, we strip the trailing slash from dataPath since `path` may be input as a file formatted path even if it is a folder (i.e. missing trailing slash)
    dirDataPath = ofFilePath::addTrailingSlash(dirDataPath);

    auto relativeDirDataPath = ofFilePath::makeRelative(std::filesystem::current_path().string(),dataPath.string());
    relativeDirDataPath  = ofFilePath::addTrailingSlash(relativeDirDataPath);

    if (inputPath.string().find(dirDataPath) != 0 && inputPath.string().find(relativeDirDataPath)!=0) {
		// inputPath doesn't contain data path already, so we build the output path as the inputPath relative to the dataPath
	    if(makeAbsolute){
            outputPath = dirDataPath / inputPath;
	    }else{
            outputPath = relativeDirDataPath / inputPath;
	    }
	} else {
		// inputPath already contains data path, so no need to change
		outputPath = inputPath;
	}

    // finally, if we do want an absolute path and we don't already have one
	if(makeAbsolute){
	    // then we return the absolute form of the path
	    try {
            auto outpath = std::filesystem::canonical(std::filesystem::absolute(outputPath)).make_preferred();
            if(std::filesystem::is_directory(outpath) && hasTrailingSlash){
                return ofFilePath::addTrailingSlash(outpath.string());
            }else{
                return outpath.string();
            }
	    }
	    catch (std::exception &) {
            return std::filesystem::absolute(outputPath).string();
	    }
	}else{
		// or output the relative path
        return outputPath.string();
	}
}


//----------------------------------------
int ofToInt(const string& intString) {
	return ofTo<int>(intString);
}


//--------------------------------------------------
string ofVAArgsToString(const char * format, ...){
	va_list args;
	va_start(args, format);
	char buf[256];
	size_t n = std::vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);

	// Static buffer large enough?
	if (n < sizeof(buf)) {
		return{ buf, n };
	}

	// Static buffer too small
	std::string s(n + 1, 0);
	va_start(args, format);
	std::vsnprintf(const_cast<char*>(s.data()), s.size(), format, args);
	va_end(args);

	return s;
}

string ofVAArgsToString(const char * format, va_list args){
	char buf[256];
	size_t n = std::vsnprintf(buf, sizeof(buf), format, args);

	// Static buffer large enough?
	if (n < sizeof(buf)) {
		return{ buf, n };
	}

	// Static buffer too small
	std::string s(n + 1, 0);
	std::vsnprintf(const_cast<char*>(s.data()), s.size(), format, args);

	return s;
}


//--------------------------------------------------
void ofStringReplace(string& input, const string& searchStr, const string& replaceStr){
	auto pos = input.find(searchStr);
	while(pos != std::string::npos){
		input.replace(pos, searchStr.size(), replaceStr);
		pos += replaceStr.size();
		std::string nextfind(input.begin() + pos, input.end());
		auto nextpos = nextfind.find(searchStr);
		if(nextpos==std::string::npos){
			break;
		}
		pos += nextpos;
	}
}

//default ofGetTimestampString returns in this format: 2011-01-15-18-29-35-299
//--------------------------------------------------
string ofGetTimestampString(){

	string timeFormat = "%Y-%m-%d-%H-%M-%S-%i";

	return ofGetTimestampString(timeFormat);
}

//specify the string format - eg: %Y-%m-%d-%H-%M-%S-%i ( 2011-01-15-18-29-35-299 )
//--------------------------------------------------
string ofGetTimestampString(const string& timestampFormat){
	std::stringstream str;
	auto now = std::chrono::system_clock::now();
	auto t = std::chrono::system_clock::to_time_t(now);    std::chrono::duration<double> s = now - std::chrono::system_clock::from_time_t(t);
    int ms = s.count() * 1000;
	auto tm = *std::localtime(&t);
	constexpr int bufsize = 256;
	char buf[bufsize];

	// Beware! an invalid timestamp string crashes windows apps.
	// so we have to filter out %i (which is not supported by vs)
	// earlier.
	auto tmpTimestampFormat = timestampFormat;
	ofStringReplace(tmpTimestampFormat, "%i", ofToString(ms, 3, '0'));

	if (strftime(buf,bufsize, tmpTimestampFormat.c_str(),&tm) != 0){
		str << buf;
	}
	auto ret = str.str();


    return ret;
}


ofUTF8Iterator::ofUTF8Iterator(const string & str){
	try{
		utf8::replace_invalid(str.begin(),str.end(),back_inserter(src_valid));
	}catch(...){
	}
}

utf8::iterator<std::string::const_iterator> ofUTF8Iterator::begin() const{
	try {
		return utf8::iterator<std::string::const_iterator>(src_valid.begin(), src_valid.begin(), src_valid.end());
	}
	catch (...) {
		return utf8::iterator<std::string::const_iterator>();
	}
}

utf8::iterator<std::string::const_iterator> ofUTF8Iterator::end() const{
	try {
		return utf8::iterator<std::string::const_iterator>(src_valid.end(), src_valid.begin(), src_valid.end());
	}
	catch (...) {
		return utf8::iterator<std::string::const_iterator>();
	}
}

utf8::iterator<std::string::const_reverse_iterator> ofUTF8Iterator::rbegin() const {
	try {
		return utf8::iterator<std::string::const_reverse_iterator>(src_valid.rbegin(), src_valid.rbegin(), src_valid.rend());
	}
	catch (...) {
		return utf8::iterator<std::string::const_reverse_iterator>();
	}
}

utf8::iterator<std::string::const_reverse_iterator> ofUTF8Iterator::rend() const {
	try {
		return utf8::iterator<std::string::const_reverse_iterator>(src_valid.rbegin(), src_valid.rbegin(), src_valid.rend());
	}
	catch (...) {
		return utf8::iterator<std::string::const_reverse_iterator>();
	}
}



//--------------------------------------------------
// helper method to get locale from name
static std::locale getLocale(const string & locale) {
std::locale loc;
#if defined(TARGET_WIN32) && !_MSC_VER
	static bool printonce = true;
	if( printonce ){
		std::string current( setlocale(LC_ALL,NULL) );
		setlocale (LC_ALL,"");
		ofLogWarning("ofUtils") << "std::locale not supported. Using C locale  :" << current ;
		printonce = false;
	}
#else
	try {
		loc = std::locale(locale.c_str());
	}
	catch (...) {
		ofLogWarning("ofUtils") << "Couldn't create locale " << locale << " using default, " << loc.name();
	}
#endif
	return loc;
}

//--------------------------------------------------
string ofToLower(const string & src, const string & locale){
	std::string dst;
	std::locale loc = getLocale(locale);
	try{
		for(auto c: ofUTF8Iterator(src)){
			utf8::append(std::tolower<wchar_t>(c, loc), back_inserter(dst));
		}
	}catch(...){
	}
	return dst;
}