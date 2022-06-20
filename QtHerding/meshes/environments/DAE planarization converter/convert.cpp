#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

int main(int argc, char* argv[])
{
  if (argc > 2)
  {
    cout << "Too many arguments!" << endl;
    return 0;
  }
  string fn_in((argc == 2) ? argv[1] : "inp.dae");
  string fn_out(fn_in);
  size_t ext_pos = fn_out.find_last_of(".");
  if (ext_pos == string::npos) ext_pos = fn_out.size();
  fn_out.erase(ext_pos, 4);
  fn_out.append("_2dplanar.dae");
  cout << "Converting file " << fn_in << " into planar " << fn_out << "...";
  ifstream infile(fn_in.c_str());
  ofstream outfile(fn_out.c_str());
  string s;
  while (getline(infile, s))
  {
    string temp(s);
    string orig(s);
    size_t pos = s.find("float_array");
    if (pos != string::npos)
    {
      pos = s.find(">");
      s.erase(pos + 1, s.size() - pos - 1);
      temp.erase(temp.size() - 14, 14);
      temp.erase(0, pos + 1);
      //cout << temp << endl;
      stringstream ss(temp);
      stringstream ssout;
      ssout << s;
      double d;
      int count = 0;
      while (ss >> d)
      {
        if (count == 1)  // Y coordinate is to be zeroed
        {
          d = 0;
        }
        ssout << setprecision(7) << d << " ";
        count++;
        count %= 3;
      }
      s = ssout.str();
      s.erase(s.size() - 1, 1);
      s.append("</float_array>");
    }
    outfile << s << endl;
  }
  cout << " done" << endl;
  return 0;
}
