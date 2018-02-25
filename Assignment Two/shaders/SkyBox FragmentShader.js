var skybox_fs = `

precision mediump float;
		
uniform samplerCube tCube;
varying vec3 vWorldPosition;

void main() 
{
	gl_FragColor = textureCube( tCube, vec3(  vWorldPosition ) );
}`;