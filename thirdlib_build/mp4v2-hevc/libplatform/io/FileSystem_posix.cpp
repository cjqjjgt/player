#include "libplatform/impl.h"
#include <sys/stat.h>

namespace mp4v2 { namespace platform { namespace io {

///////////////////////////////////////////////////////////////////////////////

bool
FileSystem::exists( string path_ )
{
    struct stat buf;
	printf("mp4v2-----stat exists \n");
    return stat( path_.c_str(), &buf ) == 0;
}

///////////////////////////////////////////////////////////////////////////////

bool
FileSystem::isDirectory( string path_ )
{
    struct stat buf;
	printf("mp4v2-----stat isDirectory \n");
    if( stat( path_.c_str(), &buf ))
        return false;
    return S_ISDIR( buf.st_mode );
}

///////////////////////////////////////////////////////////////////////////////

bool
FileSystem::isFile( string path_ )
{
    struct stat buf;
	printf("mp4v2-----stat isFile \n");
    if( stat( path_.c_str(), &buf ))
        return false;
    return S_ISREG( buf.st_mode );
}

///////////////////////////////////////////////////////////////////////////////

bool
FileSystem::getFileSize( string path_, File::Size& size_ )
{
    size_ = 0;
	#if 0
    struct stat buf;
	if( stat( path_.c_str(), &buf ))
        return true;
    size_ = buf.st_size;
	#endif
	FILE *fp;
	//printf("file path :%s\n", path_);
    fp = fopen(path_.c_str(),"r");
    if(NULL == fp)
	 {  
	   printf("Read file error\n");
	   return true;
    }
    else
    {
       fseek(fp,0,SEEK_END); 
	   size_ = ftell(fp);
	   fclose(fp);
	   printf("mp4v2--- open file success\n");
    }
    return false;
}

///////////////////////////////////////////////////////////////////////////////

bool
FileSystem::rename( string from, string to )
{
    return ::rename( from.c_str(), to.c_str() ) != 0;
}

///////////////////////////////////////////////////////////////////////////////

string FileSystem::DIR_SEPARATOR  = "/";
string FileSystem::PATH_SEPARATOR = ":";

///////////////////////////////////////////////////////////////////////////////

}}} // namespace mp4v2::platform::io
