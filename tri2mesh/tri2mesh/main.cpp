#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
#include <vector>

//#define USE_OPENMESH
#ifdef USE_OPENMESH
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;
#endif // USE_OPENMESH

using namespace std;

double str2num(string s);
void read_node(string file, vector<double> &nodes);
void read_ele(string file, vector<int> &eles);
void saveAsMesh(const string &file, const vector<double> &nodes, const vector<int> &eles);

int main()
{
	L:
	string filename, nodefilename, elefilename, nodefile, elefile;

	cout<<"input file name (.node & .ele do not need): ";
	cin>>filename;
	nodefilename = filename + ".node";
	elefilename  = filename + ".ele";
	cout<<endl;
	nodefile = "../triangle/" + nodefilename;
	elefile  = "../triangle/" + elefilename;

	vector<double> nodes;
	vector<int> eles;
	read_node(nodefile, nodes);
	read_ele(elefile, eles);
#ifdef USE_OPENMESH


	MyMesh mesh;
	vector<MyMesh::VertexHandle> vhVec;
	for (int i=0; i<nodes.size(); i++)
	{
		MyMesh::Point p(nodes.at(i), nodes.at(i+1), 0.0);
		vhVec.push_back( mesh.add_vertex(p) );
		i++;
	}
	vector<MyMesh::VertexHandle> fhVec;
	for (int i=0; i<eles.size(); i++)
	{
		fhVec.clear();
		MyMesh::VertexHandle vh[3];
		vh[0] = vhVec.at( eles.at(i) );
		vh[1] = vhVec.at( eles.at(i+1) );
		vh[2] = vhVec.at( eles.at(i+2) );
		for (int j=0; j<3; j++)
		{
			fhVec.push_back(vh[j]);
		}
		mesh.add_face(fhVec);
		i = i+2;
	}
	OpenMesh::IO::write_mesh(mesh, "mesh.obj");

#else
	saveAsMesh("../out.obj", nodes, eles);
#endif // USE_OPENMESH

	cout<<endl<<endl<<"  go on  ???   (1 is yes)"<<endl;
	int iscontinue;
	cin>>iscontinue;
	if (iscontinue == 1)
	{
		goto L;
	}
	
	return 0;
}


void read_node(string file, vector<double> &nodes)
{
	ifstream infile; 
	infile.open(file.data()); 
	assert(infile.is_open());

	int l=1;
	int cnt;
	string s;
	while (getline(infile, s))
	{

		string s0;
		string sx, sy;
		sx.clear();
		sy.clear();
		s0.clear();
		if (l == 1)                    //第一行的第一个字符必须为数字
		{
			for (int i=0; i<s.length(); i++)     
			{
				if (s.at(i) == ' ')
				{
					break;
				}
				s0 += s.at(i);
			}
			cnt = atoi(s0.c_str());
			cout<<"number of vertices : "<<cnt<<endl;
		}
		else                                         //从第二行开始，不准许有空行或者注释行，直至点坐标结束
		{
			int i, j;
			for (i=0; i<s.length(); i++)
			{
				if (s.at(i) == ' ')
				{
					continue;
				}
				for (j=i; j<s.length(); j++)     //read the index of vertex
				{
					if (s.at(j) == ' ')
					{
						break;
					}
					s0 += s.at(j);
				}
				i = j;
				for (j=i; j<s.length(); j++)
				{
					if (s.at(j) != ' ')
					{
						break;
					}
				}
				i = j;
				for (j=i; j<s.length(); j++)
				{
					if (s.at(j) == ' ')
					{
						break;
					}
					sx += s.at(j);
				}
				i = j;
				for (j=i; j<s.length(); j++)
				{
					if (s.at(j) != ' ')
					{
						break;
					}
				}
				i = j;
				for (j=i; j<s.length(); j++)
				{
					if (s.at(j) == ' ')
					{
						break;
					}
					sy += s.at(j);
				}
				break;                 //skip the remaining chars
			}
			double x, y;
			x = str2num(sx);
			y = str2num(sy);
			nodes.push_back(x);
			nodes.push_back(y);
		}
		//cout<<l-1<<"   sx = "<<sx<<"   sy = "<<sy<<endl;
		l++;
		if (l-1 > cnt)
		{
			break;
		}
	}
}

void read_ele(string file, vector<int> &eles)
{
	ifstream infile; 
	infile.open(file.data()); 
	assert(infile.is_open());

	int l=1;
	int cnt;
	string s;
	while (getline(infile, s))
	{

		string s0;
		string sa, sb, sc;
		s0.clear();
		sa.clear();
		sb.clear();
		sc.clear();
		if (l == 1)                    //第一行的第一个字符必须为数字
		{
			for (int i=0; i<s.length(); i++)     
			{
				if (s.at(i) == ' ')
				{
					break;
				}
				s0 += s.at(i);
			}
			cnt = atoi(s0.c_str());
			cout<<"number of triangles : "<<cnt<<endl;
		}
		else 
		{
			int i, j;
			for (i=0; i<s.length(); i++)
			{
				if (s.at(i) == ' ')
				{
					continue;
				}

				for (j=i; j<s.length(); j++)     //read the index of triangle
				{
					if (s.at(j) == ' ')
					{
						break;
					}
					s0 += s.at(j);
				}
				i = j;
				for (j=i; j<s.length(); j++)
				{
					if (s.at(j) != ' ')
					{
						break;
					}
				}
				i = j;

				for (j=i; j<s.length(); j++)
				{
					if (s.at(j) == ' ')
					{
						break;
					}
					sa += s.at(j);
				}
				i = j;
				for (j=i; j<s.length(); j++)
				{
					if (s.at(j) != ' ')
					{
						break;
					}
				}
				i = j;

				for (j=i; j<s.length(); j++)
				{
					if (s.at(j) == ' ')
					{
						break;
					}
					sb += s.at(j);
				}
				i = j;
				for (j=i; j<s.length(); j++)
				{
					if (s.at(j) != ' ')
					{
						break;
					}
				}
				i = j;

				for (j=i; j<s.length(); j++)
				{
					if (s.at(j) == ' ')
					{
						break;
					}
					sc += s.at(j);
				}
				break;                 //skip the remaining chars
			}
			//cout<<sa<<"  "<<sb<<"  "<<sc<<endl;
			eles.push_back(atoi(sa.c_str())-1);
			eles.push_back(atoi(sb.c_str())-1);
			eles.push_back(atoi(sc.c_str())-1);

		}
		l++;
		if (l-1 > cnt)
		{
			break;
		}
	}
}

double str2num(string s)
{
	if (s.length() == 0)
	{
		cout<<"error input"<<endl;
		return 0;
	}
	//cout<<"init : "<<s<<endl;
	bool isnegative = false;
	if (s.at(0) == '-')
	{
		isnegative = true;
		s = s.substr(1);          //delete first char
	}

	string lstr, rstr;
	int i;
	for (i=0; i<s.length(); i++)
	{
		if (s.at(i) == '.')
		{
			break;                //stop when meet a dot
		}
		lstr += s.at(i);
	}
	for (int j=i+1; j<s.length(); j++)
	{
		if (s.at(j) != ' ' || s.at(j) != '\n')
		{
			rstr += s.at(j);
		}
	}
	//cout<<"temp : "<<s<<endl;
	//cout<<l<<endl<<r<<endl;
	if (rstr.length() > 6)
	{
		rstr = rstr.substr(0, 6);
	}
	int lnum = atoi(lstr.c_str());
	int rnum = atoi(rstr.c_str());
	double res;
	if (rstr.length() == 0)
	{
		res = lnum;
	}
	else
	{
		res = lnum + rnum*1.0/(pow(10.0, rstr.length()*1.0));
	}

	if (isnegative)
	{
		return -res;
	}
	else
	{
		return res;
	}
}

void saveAsMesh(const string &file, const vector<double> &nodes, const vector<int> &eles)
{
	string outDir = file;
	std::ofstream out = std::ofstream(outDir);
	for (int i = 0; i < nodes.size(); i++)
	{
		out << "v ";
		out << nodes[i] << " " << nodes[i+1] << " " << 0.0;
		out << endl;
		i++;
	}
	for (int i = 0; i < eles.size(); i++)
	{
		out << "f ";
		out << eles[i] + 1 << " " << eles[i+1] + 1 << " " << eles[i+2] + 1;
		out << endl;
		i = i + 2;
	}
	out.close();
}