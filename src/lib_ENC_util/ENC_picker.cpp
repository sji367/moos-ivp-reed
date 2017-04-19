#include <ENC_picker.h>

ENC_Picker::ENC_Picker()
{
  RNC.clear();
  setOrigin(0,0);
  root_directory = "src/ENCs/";//"/home/sji367/moos-ivp/moos-ivp-reed/src/ENCs";
  csvfile = root_directory+"ENCList.csv";
}

ENC_Picker::ENC_Picker(double latOrigin,double lonOrigin)
{
  RNC.clear();
  setOrigin(latOrigin, lonOrigin);
  root_directory = "src/ENCs/";//"~/moos-ivp/moos-ivp-reed/src/ENCs";
  csvfile = root_directory+"ENCList.csv";
}

ENC_Picker::ENC_Picker(double latOrigin,double lonOrigin, string root_dir)
{
  RNC.clear();
  setOrigin(latOrigin, lonOrigin);
  root_directory = root_dir;
  csvfile = root_directory+"ENCList.csv";
}

ENC_Picker::ENC_Picker(double latOrigin,double lonOrigin, string root_dir, string csvfilename)
{
  RNC.clear();
  setOrigin(latOrigin, lonOrigin);
  root_directory = root_dir;
  csvfile = root_directory+"/"+csvfilename;
}

void ENC_Picker::parse_csv()
{
  // Make sure that the RNC map is clear
  RNC.clear();
  
  strtk::token_grid::options option;
  option.column_delimiters = ",";

  // Read in the file
  strtk::token_grid grid(csvfile, option);
  strtk::token_grid::row_type row;

  // Skip the headers and any chart that does not have an associated scale
  for(size_t i = 1; i < grid.row_count(); ++i)
    {
      row = grid.row(i);
      if (row.get<string>(1)!="nan")
	{
	  RNC[row.get<string>(0)] = row.get<int>(1);
	}
    }
}

void ENC_Picker::build_ENC_outlines()
{
  GDALAllRegister();
  
  GDALDataset* ds;
  OGRLayer* layer;
  OGRFeature* feat;
  OGRGeometry* geom;
  OGRPolygon* newPoly;

  string source_indication;
  
  vector<string> path, temp_split, temp_split2;
  fs::path root = root_directory;
  Dir_Walk walk = Dir_Walk(root, ".000");
  walk.find_all_ext(path, ENC_Names);
  for (int i =0; i<path.size(); i++)
    {
      ds = (GDALDataset*) GDALOpenEx( path[i].c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
      // Check if it worked
      if( ds == NULL )
	{
	  printf( "Open failed.\n" );
	  exit( 1 );
	}
      layer = ds->GetLayerByName("M_NPUB");
      layer->ResetReading();
      feat = layer->GetNextFeature();
      geom = feat->GetGeometryRef();
      newPoly = (OGRPolygon *) geom;
      ENC_Outline.push_back(newPoly);
      source_indication = feat->GetFieldAsString("SORIND");
      // Parse the information to get the RNC chart number
      //  In the format:
      //    Country, Authority, Source, ID-Code
      split(temp_split, source_indication, ',');
      split(temp_split2, temp_split.back(), ' ');
      chart.push_back(temp_split2.back());
    }
}

void ENC_Picker::pick_ENC(string &chart_name, int &chart_scale)
{
  string ENC;
  int temp_scale;
  chart_scale = -1;

  // Get outlines of the ENCs and parse the csv file that relates the RNCs
  //  with scales
  parse_csv();
  build_ENC_outlines();

  // determine which ENCs the origin lies within
  for (int i =0; i<ENC_Outline.size(); i++)
    {
      if (point->Within(ENC_Outline[i]))
	{
	  ENC= chart[i];
	  map<string,int>::iterator search;
	  search = RNC.find(ENC);
	  // If the origin lies within the ENC and its a smaller scale, then
	  //  store the ENC's name and scale
	  if(search != RNC.end())
	    {
	      temp_scale = search->second;
	      if ((temp_scale < chart_scale)||(chart_scale==-1))
		{
		  chart_name = ENC_Names[i];
		  chart_scale = temp_scale;
		  //cout << "Inside: " << ENC_Names[i] << " " << temp_scale << endl;
		}
	    }
	  else 
	    cout << ENC << " not found" << endl;
	}
    }
}

void split(vector<string>& parsed, const string& s, const char c)
{
  unsigned int i = 0;
  unsigned int j = s.find(c);
  while (j < s.length()) {
    parsed.push_back(s.substr(i, j - i));
    i = ++j;
    j = s.find(c, j);
    if (j >= s.length()) {
      parsed.push_back(s.substr(i, s.length()));
      break;
    }
  }
}
