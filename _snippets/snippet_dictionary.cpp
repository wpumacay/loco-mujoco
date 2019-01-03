
#include <map>
#include <string>
#include <iostream>


struct vec3
{
	float x;
	float y;
	float z;
};

struct vec4
{
	float x;
	float y;
	float z;
	float w;
};

std::string tostring( const vec3& vec )
{
	std::string _str;
	
	_str += "[ ";
	_str += std::to_string( vec.x ) + ", ";
	_str += std::to_string( vec.y ) + ", ";
	_str += std::to_string( vec.z ) + " ]";

	return _str;
}

std::string tostring( const vec4& vec )
{
	std::string _str;
	
	_str += "[ ";
	_str += std::to_string( vec.x ) + ", ";
	_str += std::to_string( vec.y ) + ", ";
	_str += std::to_string( vec.z ) + ", ";
	_str += std::to_string( vec.w ) + " ]";

	return _str;
}

class Dictionary
{
	private :
	
	std::map< std::string, vec3 > m_vec3s;
	std::map< std::string, vec4 > m_vec4s;
	
	public :

	void set( const std::string& field, const vec3& vec )
	{
		m_vec3s[ field ] = vec;
	}

	void set( const std::string& field, const vec4& vec )
	{
		m_vec4s[ field ] = vec;
	}
	
	vec3& getVec3( const std::string& field )
	{
		return m_vec3s[ field ];
	}

	vec4& getVec4( const std::string& field )
	{
		return m_vec4s[ field ];
	}
};



int main()
{
	Dictionary _dict;
	
	vec3 _v1 = { 0.0f, 0.0f, 1.0f };
	vec4 _v2 = { 0.0f, 0.0f, 0.0f, 1.0f };

	_dict.set( "position", _v1 );
	_dict.set( "rotation", _v2 );
	
	std::cout << "position: " << tostring( _dict.getVec3( "position" ) ) << std::endl;
	std::cout << "rotation: " << tostring( _dict.getVec4( "rotation" ) ) << std::endl;

	return 0;
}

