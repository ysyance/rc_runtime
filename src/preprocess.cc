#include "preprocess.hh"




/******************************************/
/******************************************/
/*Section 02  : Process data file...      */
/*Author      : Wang Zhen                 */
/*Create Date : 2014.12.29                */
/*Last Update : 2014.12.29                */
/******************************************/
/******************************************/

bool regex_data_match(char *program_directory, char **program_name)
{
    char *pattern = (char *) std::string("(.+).tid$").c_str();
    char errbuf[1024];
    regex_t reg;
    int err = -1,nm = 2;
    regmatch_t pmatch[2];

    if(regcomp(&reg, pattern, REG_EXTENDED) < 0)
    {
        regerror(err, &reg, errbuf, sizeof(errbuf));
        std::cout << "err: " << errbuf << std::endl;
    }
    err = regexec(&reg,  program_directory, nm, pmatch, 0);
    if( !err )
    {
        for(int i = 1; i < 2 && pmatch[i].rm_so != -1; ++i)
        {
            int len = pmatch[i].rm_eo - pmatch[i].rm_so;
            if(len)
            {
                *program_name = new char[len + 1](); //each member is '\0'
                memcpy(*program_name,  program_directory+pmatch[i].rm_so, len);
            }
        }
        return true;
    }
    else
        return false;
}

#define pathmax 50

static char *data_fullpath;
static size_t data_pathlen;
static int data_flag = 1;


int data_myftw(char *robot_name, const char *project_directory, robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable)
{
    data_pathlen = pathmax;
    data_fullpath = new char[pathmax];
    if(data_pathlen <= strlen(robot_name))
    {
        data_pathlen = strlen(robot_name) * 2;
        if( (data_fullpath = (char *)realloc(data_fullpath, data_pathlen)) == NULL )
            std::cout << "realloc failed" << std::endl;
    }
    strcpy(data_fullpath, robot_name);
    return(data_dopath(project_directory, symtable_of_symtable));
}

int data_dopath(const char *project_directory, robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable)
{
    robot_data_file_process::symbol_c *tree_root;
    char *program_name;
    struct stat statbuf;
    struct dirent *dirp;
    DIR *dp;
    int ret = 0, n;
    if(lstat(data_fullpath, &statbuf) < 0)
    {
        std::cout << "stat error for " << data_fullpath << std::endl;
        return -1;
    }
    if(S_ISDIR(statbuf.st_mode) == 0)
    {
        std::cout << data_fullpath << " is a file " << std::endl;

        if(regex_data_match(data_fullpath, &program_name)) //.tid data file
        {
            /* 1st Pass */
            if (data_stage1_2(data_fullpath, &tree_root) < 0)
                return -2;

            /* 4rd Pass */
            if (robot_data_file_process::data_stage4(&program_name, &tree_root, symtable_of_symtable) < 0) //Pay attention to the fisrt parameter!
                return -3;
        }
        return 0;
    }
    n = strlen(data_fullpath);
    if(n + NAME_MAX + 2 > data_pathlen)
    {
        data_pathlen *= 2;
        if( (data_fullpath = (char *)realloc(data_fullpath, data_pathlen)) == NULL)
            std::cout << "realloc failed " << std::endl;
    }
    data_fullpath[n++] = '/';
    data_fullpath[n] = 0; //equals '\0'
    if((dp = opendir(data_fullpath)) == NULL)
    {
        std::cout << "can't read directory " << data_fullpath << std::endl;
        return -4;
    }

    if( (!strncmp(data_fullpath,project_directory,8) && data_flag) || !strncmp(data_fullpath,project_directory,strlen(project_directory)))
    {
        data_flag = 0;
        while((dirp = readdir(dp)) != NULL)
        {
            if(strcmp(dirp->d_name, ".") == 0 || strcmp(dirp->d_name, "..") == 0)
                continue;
            strcpy(&data_fullpath[n], dirp->d_name);
            ret = data_dopath(project_directory, symtable_of_symtable);
        }
        data_fullpath[n-1] = 0; //equals '\0'
    }

    if(closedir(dp) < 0)
        std::cout << "can't close directory " << data_fullpath << std::endl;
    return(ret);
}

/******************************************/
/******************************************/
/*Section 03  : Process program file...   */
/*Author      : Wang Zhen                 */
/*Create Date : 2014.12.29                */
/*Last Update : 2014.12.29                */
/******************************************/
/******************************************/
bool regex_program_match(char *program_directory, char **program_name)
{
    char *pattern = (char *) std::string("(.+).tip$").c_str();
    char errbuf[1024];
    regex_t reg;
    int err = -1,nm = 2;
    regmatch_t pmatch[2];

    if(regcomp(&reg, pattern, REG_EXTENDED) < 0)
    {
        regerror(err, &reg, errbuf, sizeof(errbuf));
        std::cout << "err: " << errbuf << std::endl;
    }
    err = regexec(&reg,  program_directory, nm, pmatch, 0);
    if( !err )
    {
        for(int i = 1; i < 2 && pmatch[i].rm_so != -1; ++i)
        {
            int len = pmatch[i].rm_eo - pmatch[i].rm_so;
            if(len)
            {
                *program_name = new char[len + 1](); //each member is '\0'
                memcpy(*program_name,  program_directory+pmatch[i].rm_so, len);
            }
        }
        return true;
    }
    else
        return false;
}

//#define pathmax 50

static char *program_fullpath;
static size_t program_pathlen;
static int program_flag = 1;

int program_myftw(char *robot_name, const char *project_directory, robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable, robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable)
{
    program_pathlen = pathmax;
    program_fullpath = new char[pathmax];
    if(program_pathlen <= strlen(robot_name))
    {
        program_pathlen = strlen(robot_name) * 2;
        if( (program_fullpath = (char *)realloc(program_fullpath, program_pathlen)) == NULL )
            std::cout << "realloc failed" << std::endl;
    }
    strcpy(program_fullpath, robot_name);
    return(program_dopath(project_directory, subprogram_symtable, symtable_of_symtable));
}

int program_dopath(const char *project_directory, robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable, robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable)
{
    robot_program_file_process::symbol_c *tree_root;
    char *program_name;
    struct stat statbuf;
    struct dirent *dirp;
    DIR *dp;
    int ret = 0, n;
    if(lstat(program_fullpath, &statbuf) < 0)
    {
        std::cout << "stat error for " << program_fullpath << std::endl;
        return -1;
    }
    if(S_ISDIR(statbuf.st_mode) == 0)
    {
        std::cout << program_fullpath << " is a file " << std::endl;

        if(regex_program_match(program_fullpath, &program_name)) //.tid data file
        {
            /* 1st Pass */
            if (program_stage1_2(program_fullpath, &tree_root) < 0)
                return -2;

            /* 4rd Pass */
            if (robot_program_file_process::program_stage4(&program_name, &tree_root, subprogram_symtable, symtable_of_symtable) < 0) //Pay attention to the fisrt parameter!
                return -3;
        }
        return 0;
    }
    n = strlen(program_fullpath);
    if(n + NAME_MAX + 2 > program_pathlen)
    {
        program_pathlen *= 2;
        if( (program_fullpath = (char *)realloc(program_fullpath, program_pathlen)) == NULL)
            std::cout << "realloc failed " << std::endl;
    }
    program_fullpath[n++] = '/';
    program_fullpath[n] = 0; //equals '\0'
    if((dp = opendir(program_fullpath)) == NULL)
    {
        std::cout << "can't read directory " << program_fullpath << std::endl;
        return -4;
    }

    if( (!strncmp(program_fullpath,project_directory,8) && program_flag) || !strncmp(program_fullpath,project_directory,strlen(project_directory)))
    {
        program_flag = 0;
        while((dirp = readdir(dp)) != NULL)
        {
            if(strcmp(dirp->d_name, ".") == 0 || strcmp(dirp->d_name, "..") == 0)
                continue;
            strcpy(&program_fullpath[n], dirp->d_name);
            ret = program_dopath(project_directory, subprogram_symtable, symtable_of_symtable);
        }
        program_fullpath[n-1] = 0; //equals '\0'
    }

    if(closedir(dp) < 0)
        std::cout << "can't close directory " << program_fullpath << std::endl;
    return(ret);
}

void error_exit(const char *file_name, int line_no) {
    std::cerr << "\nInternal program error in file " << file_name
        << " at line " << line_no << "\n\n\n";
    exit(EXIT_FAILURE);
}