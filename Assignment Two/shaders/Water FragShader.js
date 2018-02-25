var water_fs = ` 

precision mediump float;

uniform samplerCube envMap;

varying vec3 vI, vWorldNormal;

void main()
{
  vec3 reflection = reflect( vI, vWorldNormal );

  vec4 waterColor = vec4(0, 0.7, 1, 1.0); 
  vec4 envColor = textureCube( envMap, vec3( -reflection.x, reflection.yz ) );
  vec4 finalColor = mix(envColor, waterColor, 0.3);

  gl_FragColor = finalColor;
}`;