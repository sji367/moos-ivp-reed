#include "boost/filesystem.hpp"
#include <iostream>
#include <vector>

using namespace std;
namespace fs = ::boost::filesystem;

#ifndef DIR_WALK_H_
#define DIR_WALK_H_

class Dir_Walk
{
public:
	// Constructor/Deconstructor
	Dir_Walk() {setRootExt( "../../src/ENCs/", ".000"); }; 
	Dir_Walk(fs::path Root) {setRootExt(Root, ".000"); };
	Dir_Walk(fs::path Root, string Ext) {setRootExt(Root, Ext); };
	~Dir_Walk() {};

	// return the filenames of all files that have the specified extension
	// in the specified directory and all subdirectories
	void find_all_ext(vector<string>& path_names, vector<string>& file_names);

	void setPath(fs::path Root) {root = Root; };
	void setExt(string Ext) {ext = Ext; };
	void setRootExt(fs::path Root, string Ext) {setPath(Root); setExt(Ext); };
private:
	fs::path root;
	string ext;
};

#endif /* GEODESY_H_ */
