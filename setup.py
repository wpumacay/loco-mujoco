#!/usr/bin/env python

import os
import sys
import glob
import subprocess

from setuptools import find_packages, setup, Extension
from setuptools.command.build_ext import build_ext
from setuptools.command.install import install

VERSION_MAJOR = 0
VERSION_MINOR = 0
VERSION_MICRO = 1
VERSION = '%d.%d.%d' % ( VERSION_MAJOR, VERSION_MINOR, VERSION_MICRO )
PREFIX = 'wp_loco_mujoco_%s_' % ( VERSION )

def BuildBindings( sourceDir, buildDir, cmakeArgs, buildArgs, env ):
    if not os.path.exists( buildDir ) :
        os.makedirs( buildDir )

    subprocess.call( ['cmake', sourceDir] + cmakeArgs, cwd=buildDir, env=env )
    subprocess.call( ['cmake', '--build', '.'] + buildArgs, cwd=buildDir )

# @hack: get installation path: https://stackoverflow.com/questions/36187264/how-to-get-installation-directory-using-setuptools-and-pkg-ressources
def GetInstallationDir() :
    py_version = '%s.%s' % ( sys.version_info[0], sys.version_info[1] )
    install_path_candidates = ( path % (py_version) for path in (
                        sys.prefix + '/lib/python%s/dist-packages/',
                        sys.prefix + '/lib/python%s/site-packages/',
                        sys.prefix + '/local/lib/python%s/dist-packages/',
                        sys.prefix + '/local/lib/python%s/site-packages/',
                        '/Library/Python/%s/site-packages/' ) )
    for path_candidate in install_path_candidates :
        if os.path.exists( path_candidate ) :
            return path_candidate

    print( 'ERROR >>> No installation path found', file=sys.stderr )
    return None

def GetFilesUnderPath( path, dst_path, extension ) :
    cwd_path = os.getcwd()
    target_path = os.path.join( cwd_path, path )
    if not os.path.exists( target_path ) :
        return ( '', [] )

    os.chdir( target_path )
    files = glob.glob( '**/*.%s' % ( extension ), recursive=True )
    files_paths = [ os.path.join( target_path, fpath ) for fpath in files ]
    os.chdir( cwd_path )
    return ( PREFIX + dst_path, files_paths )

# @hack: get resources path (should replace with proper find functionality)
def GetResourcesDir() :
    py_version = '%s.%s' % ( sys.version_info[0], sys.version_info[1] )
    install_path_candidates = ( path % (py_version) for path in (
                        sys.prefix + '/lib/python%s/dist-packages/',
                        sys.prefix + '/lib/python%s/site-packages/',
                        sys.prefix + '/local/lib/python%s/dist-packages/',
                        sys.prefix + '/local/lib/python%s/site-packages/',
                        '/Library/Python/%s/site-packages/' ) )

    for path_candidate in install_path_candidates :
        if os.path.exists( path_candidate ) :
            if path_candidate.find( 'local' ) != -1 :
                return sys.prefix + '/local/' + PREFIX + 'res/'
            else :
                return sys.prefix + '/' + PREFIX + 'res/'

    print( 'ERROR >>> No resources path found', file=sys.stderr )
    return None

class CMakeExtension( Extension ) :

    def __init__( self, name, sourceDir ) :
        super( CMakeExtension, self ).__init__( name, sources=[] )
        self.sourceDir = os.path.abspath( sourceDir )

class BuildCommand( build_ext ) :

    user_options = [ ( 'windowed=', None, 'Whether to build with OpenGL-GLFW backend support' ),
                     ( 'headless=', None, 'Whether to build with OpenGL-EGL backend support' ),
                     ( 'debug=', None, 'Whether to build in debug-mode or not' ) ]
    boolean_options = [ 'windowed', 'headless', 'debug' ]

    def initialize_options( self ) :
        super( BuildCommand, self ).initialize_options()
        self.windowed = 1 # Build with windowed-visualizer (openglviz-GLFW) by default
        self.headless = 1 # Build with headless-visualizer (openglviz-EGL) by default
        self.debug = 1 # Build in debug mode by default

    def run( self ) :
        try:
            _ = subprocess.check_output( ['cmake', '--version'] )
        except OSError:
            raise RuntimeError( 'CMake must be installed to build the following extensions: ' +
                                ', '.join( e.name for e in self.extensions ) )

        for _extension in self.extensions :
            self.build_extension( _extension )

    def build_extension( self, extension ) :
        global DEBUG
        _extensionFullPath = self.get_ext_fullpath( extension.name )
        _extensionDirName = os.path.dirname( _extensionFullPath )
        _extensionDirPath = os.path.abspath( _extensionDirName )

        _cfg = 'Debug' if self.debug else 'Release'
        _windowed = 'ON' if self.windowed else 'OFF'
        _headless = 'ON' if self.headless else 'OFF'
        _buildArgs = ['--config', _cfg, '--', '-j8']
        _cmakeArgs = ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + _extensionDirPath,
                      '-DCMAKE_BUILD_RPATH=' + GetInstallationDir(),
                      '-DCMAKE_INSTALL_RPATH=' + GetInstallationDir(),
                      '-DPYTHON_EXECUTABLE=' + sys.executable,
                      '-DCMAKE_BUILD_TYPE=' + _cfg,
                      '-DLOCO_CORE_RESOURCES_PATH=' + GetResourcesDir(),
                      '-DLOCO_CORE_LIBRARIES_PATH=' + GetInstallationDir(),
                      '-DLOCO_CORE_BUILD_WINDOWED_VISUALIZER=' + _windowed,
                      '-DLOCO_CORE_BUILD_HEADLESS_VISUALIZER=' + _headless,
                      '-DLOCO_CORE_BUILD_DOCS=OFF',
                      '-DLOCO_CORE_BUILD_EXAMPLES=OFF',
                      '-DLOCO_CORE_BUILD_TESTS=OFF',
                      '-DLOCO_CORE_BUILD_PYTHON_BINDINGS=ON',
                      '-DLOCO_CORE_BUILD_WITH_LOGS=OFF',
                      '-DLOCO_CORE_BUILD_WITH_TRACK_ALLOCS=OFF']

        _env = os.environ.copy()
        _env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format( _env.get( 'CXXFLAGS', '' ),
                                                                self.distribution.get_version() )

        _sourceDir = extension.sourceDir
        _buildDir = self.build_temp

        BuildBindings( _sourceDir, _buildDir, _cmakeArgs, _buildArgs, _env )

class InstallCommand( install ) :

    user_options = install.user_options + BuildCommand.user_options
    boolean_options = install.boolean_options + BuildCommand.boolean_options

    def initialize_options( self ) :
        super( InstallCommand, self ).initialize_options()
        self.windowed = 1 # Build with windowed-visualizer (openglviz-GLFW) by default
        self.headless = 1 # Build with headless-visualizer (openglviz-EGL) by default
        self.debug = 1 # Build in debug mode by default

    def run( self ) :
        self.reinitialize_command( 'build_ext', headless=self.headless, debug=self.debug, windowed=self.windowed )
        self.run_command( 'build_ext' )
        super( InstallCommand, self ).run()

with open( 'README.md', 'r' ) as fh :
    longDescriptionData = fh.read()

with open( 'requirements.txt', 'r' ) as fh :
    requiredPackages = [ line.replace( '\n', '' ) for line in fh.readlines() ]

setup(
    name                    = 'wp-loco',
    version                 = VERSION,
    description             = 'MuJoCo-backend support for Loco(locomotion framework)',
    author                  = 'Wilbert Santos Pumacay Huallpa',
    license                 = 'MIT License',
    author_email            = 'wpumacay@gmail.com',
    url                     = 'https://github.com/wpumacay/tysocMjc',
    keywords                = 'locomotion control simulation MuJoCo',
    zip_safe                = False,
    install_requires        = requiredPackages,
    package_dir             = { '' : './core/python' },
    packages                = find_packages( './core/python' ),
    data_files              = [ GetFilesUnderPath( 'core/res/templates/mjcf', 'res/templates/mjcf', 'xml' ),
                                GetFilesUnderPath( 'core/res/templates/urdf', 'res/templates/urdf', 'urdf' ),
                                GetFilesUnderPath( 'core/res/templates/rlsim', 'res/templates/rlsim', 'json' ),
                                GetFilesUnderPath( 'core/res/meshes', 'res/meshes', 'stl' ),
                                GetFilesUnderPath( 'core/res/meshes', 'res/meshes', 'dae' ),
                                GetFilesUnderPath( 'core/res/meshes', 'res/meshes', 'obj' ),
                                GetFilesUnderPath( 'core/res/xml', 'res/xml', 'xml' ),
                                GetFilesUnderPath( 'core/res/xml/baxter_meshes', 'res/xml/baxter_meshes', 'stl' ),
                                GetFilesUnderPath( 'core/res/xml/baxter_meshes', 'res/xml/baxter_meshes', 'obj' ),
                                GetFilesUnderPath( 'core/res/xml/laikago_meshes', 'res/xml/laikago_meshes', 'stl' ),
                                GetFilesUnderPath( 'core/res/xml/laikago_meshes', 'res/xml/laikago_meshes', 'obj' ),
                                GetFilesUnderPath( 'core/res/xml/nao_meshes', 'res/xml/nao_meshes', 'stl' ),
                                GetFilesUnderPath( 'core/res/xml/nao_meshes', 'res/xml/nao_meshes', 'obj' ),
                                GetFilesUnderPath( 'core/res/xml/r2d2_meshes', 'res/xml/r2d2_meshes', 'stl' ),
                                GetFilesUnderPath( 'core/res/xml/r2d2_meshes', 'res/xml/r2d2_meshes', 'obj' ),
                                GetFilesUnderPath( 'core/res/xml/sawyer_meshes', 'res/xml/sawyer_meshes', 'stl' ),
                                GetFilesUnderPath( 'core/res/xml/sawyer_meshes', 'res/xml/sawyer_meshes', 'obj' ) ],
    ext_modules             = [
                                CMakeExtension( 'loco_sim', '.' )
                              ],
    cmdclass                = {
                                'build_ext': BuildCommand,
                                'install': InstallCommand
                              }
)