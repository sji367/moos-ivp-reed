#include <iostream>  // for cout
#include <numeric>   // for adjacent_difference
#include <vector>    // for vector
#include <stdlib.h>  // for rand
#include <algorithm> // for all_of

#define MAX_NUM 360

using namespace std;

// Driver function to sort the 2D vector
// on basis of a particular column
bool sortcol(const vector<double>& v1, const vector<double>& v2);

// Prints a 2D vector
void print2D_vect(vector<vector<double>> vect);

// Returns the index of the vector that is more than 5
double morethan5(vector<double> vect);

// If gap is 5 or greater interpolate between the points, overwise
//  store the angle. Each row consists of:
//      [angle, vertex_index, interpolation_index]
void store_angle(double ang, double prev_ang, double vertex_index,
		 vector<vector<double>>& vertex);

// Function to help build the rows for the vertices 2D vector
//     [angle, vertex_index, interpolation_index]
vector<double> build_row(double ang, double vertex_index, double interp_index);

// Determine the adjacent difference for first column of the 2D vertex
void adj_diff2D(vector<vector<double>>,vector<double>&);

int main()
{
  vector<vector<double>> v1;
  vector<double> result;
  //vector<double> temp;
  double ang=0;
  double prev_ang = 0;
  double index = 0;

  // Use a test vector
  vector<double> num = {10.1,12.5,14,17,4.2,2,2,2,0};

  // Store the angles and if gap between the vertices is greater than 5,
  //  interpolate
  for (int i=0; i<num.size(); ++i)
    {
      ang = num[i];
      
      if (i==0){
	prev_ang = num[num.size()-1];
      }
      else{
	prev_ang = num[i-1];
      }
      
      // If gap is 5 or greater interpolate between the points, overwise
      //  store the angle. Each row consists of:
      //      [angle, vertex_index, interpolation_index]
      store_angle(ang,prev_ang,i,v1);
    }

  // Sort the vertices by angle 
  sort(v1.begin(), v1.end(), sortcol);
  
  // Print the sorted vector
  print2D_vect(v1);

  // Determine the adjacent difference for the sorted vertex angles
  adj_diff2D(v1, result);
  
  index = morethan5(result);

  // Print results
  if (index >= 0){
    if (index == v1.size()-1)
      cout << "Gap --> min: "<< v1[index][0] << ", max: "<< v1[0][0]<< endl;
    else
      cout << "Gap --> min: "<<  v1[index][0] << ", max: "<< v1[index+1][0]<< endl;
  }
  else
    cout << "There is no gap! ERROR!!!!" << endl;
  
  return 0;
}

// Driver function to sort the 2D vector on basis of column 0
bool sortcol( const vector<double>& v1, const vector<double>& v2 )
{
  return v1[0] < v2[0];
}

// Returns the index of the vector that is more than 5
//  Should only be one gap more than 5
double morethan5(vector <double> vect)
{
  double index=-1;

  for (int i=0; i<vect.size();i++)
    {
      if (vect[i]>5)
	index = i;
    }

  return index;
}

// Prints a 2D Integer Vector
void print2D_vect(vector<vector<double>> vect)
{
  // Number of rows;
  int m = vect.size();
  
  // Number of columns
  int n = vect[0].size();

  // Displaying the 2D vector 
  cout << "The Matrix is:\n";
  for (int i=0; i<m; i++)
    {
      for (int j=0; j<n ;j++)
	cout << vect[i][j] << " ";
      cout << endl;
    }
}

// If gap is 5 or greater interpolate between the points, overwise
//  store the angle. Each row consists of:
//      [angle, vertex_index, interpolation_index]
void store_angle(double ang, double prev_ang, double vertex_index,
		  vector<vector<double>>& vertex)
{
  double diff, new_ang;
  double j;
  
  diff = (ang-prev_ang);
  
  // Account for potially going over the 0/360 boundary
  if (diff>MAX_NUM/2){
    diff -= MAX_NUM;
  }
  else if (diff<-MAX_NUM/2){
    diff += MAX_NUM;
  }

  // Linearly interpolate between the vertices if they are more than
  //  5 degrees appart.
  if (diff >= 5){
    for (j=1; j<=diff; j++) 
      {
	// Make sure the angle is not negative
	cout << (prev_ang+j) << endl;
	new_ang = fmod((prev_ang+j),MAX_NUM);
	if (new_ang < 0)
	  new_ang+=MAX_NUM;
	vertex.push_back(build_row(new_ang, vertex_index, floor(diff)-j));
      }
    if (fmod(diff,1) == 0)
      vertex.push_back(build_row(ang, vertex_index, floor(diff)+1));
  }
  else if (diff <= -5){
    for (j=diff; j<0; j++) 
      {
	cout << (prev_ang+j) << endl;
	// Make sure the angle is not negative
	new_ang = fmod((prev_ang+j),MAX_NUM);
	if (new_ang < 0)
	  new_ang+=MAX_NUM;
	vertex.push_back(build_row(new_ang, vertex_index, floor(j-diff)));
      }
    if (fmod(diff,1) == 0)
      vertex.push_back(build_row(ang, vertex_index, abs(floor(-diff))+1));
  }
  // Store the angle along with its index
  else
    vertex.push_back(build_row(ang,vertex_index,0));  
}

// Function to help build the rows for the vertices 2D vector
//     [angle, vertex_index, interpolation_index]
vector<double> build_row(double ang, double vertex_index, double interp_index)
{
  vector<double> row;
  
  row.push_back(ang);
  row.push_back(vertex_index);
  row.push_back(interp_index);

  return row;
}

// Determine the adjacent difference for first column of the 2D vertex
void adj_diff2D(vector<vector<double>> vect2D, vector<double>& result)
{
  // Takes the adjacent difference for the first column
  //       Uses difference of the indices (n+1) and n
  for (int i=0; i<vect2D.size()-1; ++i)
    {
      result.push_back(abs(vect2D[i+1][0]-vect2D[i][0]));
    }
  // Deal with boundary case (last one)
  //       Uses difference of the indices 0 and max_index
  result.push_back(abs(vect2D[0][0]+(MAX_NUM-vect2D[vect2D.size()-1][0])));
}
