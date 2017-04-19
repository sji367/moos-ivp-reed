#include "dir_walk.h"

// return the filenames of all files that have the specified extension
// in the specified directory and all subdirectories
void Dir_Walk::find_all_ext(vector<string>& path_names, vector<string>& file_names)
{
    if(!fs::exists(root) || !fs::is_directory(root))
      return;

    fs::recursive_directory_iterator it(root);
    fs::recursive_directory_iterator endit;

    while(it != endit)
    {
        if(fs::is_regular_file(*it) && it->path().extension() == ext)
	  {
	    path_names.push_back(it->path().string());
	    //cout << it->path().string() << endl;
	    file_names.push_back(it->path().filename().string());
	  }
       
        ++it;
    }
}
