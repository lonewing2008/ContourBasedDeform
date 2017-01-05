// mesh.h: interface for the mesh class.
//
//////////////////////////////////////////////////////////////////////

#ifndef AFX_MESH_H_
#define AFX_MESH_H_

#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <vector>
#include <string>

class material
{
public:
	float Ka[4];
	float Kd[4];
	float Ks[4];
	float Ns;		// shiness

	material()
	{ 
		for (size_t i=0;i<4;i++)
			Ka[i] = Kd[i] = Ks[i] = 1;
		Ns = 0;
	}
};

class mesh  
{
	class Vertex		// 儲存基本 vertex 的 property
	{
	public:
		size_t v;		// vertex (index of vList)
		size_t n;		// normal (index of nList)
		size_t t;		// texture (index of tList)
		size_t m;		// material (index of material)
		Vertex() {};
		Vertex(size_t v_index, size_t n_index, size_t t_index=0, size_t m_index=0)
		{
			v = v_index;
			n = n_index;
			t = t_index;
			m = m_index;
		}
	};

	class Vec3		// vList, nList, tList 的 structure
	{
	public:
		float ptr[3];
		Vec3 (float *v) 
		{
			for (size_t i=0;i<3;i++)
				ptr[i] = v[i];
		}
		float& operator[](size_t index)
		{
			return ptr[index];
		}
	};

	class FACE		// faceList 的 structure
	{
	public:
		Vertex v[3];		// 3 vertex for each face
		FACE (Vertex &v1, Vertex &v2, Vertex &v3) 
		{
			v[0] = v1; 
			v[1] = v2;
			v[2] = v3;
		}
		Vertex& operator[](size_t index)
		{
			return v[index];
		}
	};

	void	LoadMtl(std::string tex_file);

public:
	/////////////////////////////////////////////////////////////////////////////
	// Loading Object
	/////////////////////////////////////////////////////////////////////////////

	FILE						*scene, *texture;
	std::string					s_file, t_file;
	char						mat_file[256];		// matetial file name

	size_t						matTotal;	// total material 
	std::map<std::string,size_t>matMap;		// matMap[material_name] = material_ID
	material					mat[100];	// material ID (每個 mesh 最多有 100 種 material)	

	std::vector<Vec3>			vList;		// Vertex List (Position) - world cord.
	std::vector<Vec3>			nList;		// Normal List
	std::vector<Vec3>			tList;		// Texture List
	std::vector<FACE>			faceList;	// Face List
	
	size_t	vTotal, tTotal, nTotal, fTotal;

	void	LoadMesh(std::string scene_file);
	
	mesh();
	mesh(const char* obj_file);
	virtual ~mesh();

	void Init(const char* obj_file);
};

#endif
