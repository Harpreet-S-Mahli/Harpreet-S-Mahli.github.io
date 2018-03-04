var sphereDFVShader = `
    varying vec3 v_pos;
    uniform float time;

    void main() {
        v_pos = position;
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
    }
`;

var sphereDFFShader = `

    const int MAX_STEPS = 255;
    const float EPSILON = 0.0001;
    const float START = 0.0;
    const float END = 100.0;

    varying vec3 v_pos;
    uniform vec2 resolution;
    uniform float time;
    uniform float lightTime;
/**
 * Rotation matrix around the X axis.
 */
mat3 rotateX(float theta) {
    float c = cos(theta);
    float s = sin(theta);
    return mat3(
        vec3(1, 0, 0),
        vec3(0, c, -s),
        vec3(0, s, c)
    );
}

/**
 * Rotation matrix around the Y axis.
 */
mat3 rotateY(float theta) {
    float c = cos(theta);
    float s = sin(theta);
    return mat3(
        vec3(c, 0, s),
        vec3(0, 1, 0),
        vec3(-s, 0, c)
    );
}

/**
 * Rotation matrix around the Z axis.
 */
mat3 rotateZ(float theta) {
    float c = cos(theta);
    float s = sin(theta);
    return mat3(
        vec3(c, -s, 0),
        vec3(s, c, 0),
        vec3(0, 0, 1)
    );
}

/**
 * Constructive solid geometry intersection operation on SDF-calculated distances.
 */
float intersectSDF(float distA, float distB) {
    return max(distA, distB);
}

/**
 * Constructive solid geometry union operation on SDF-calculated distances.
 */
float unionSDF(float distA, float distB) {
    return min(distA, distB);
}

/**
 * Constructive solid geometry difference operation on SDF-calculated distances.
 */
float differenceSDF(float distA, float distB) {
    return max(distA, -distB);
}


    float cubeSDF(vec3 p) {
        // If d.x < 0, then -1 < p.x < 1, and same logic applies to p.y, p.z
        // So if all components of d are negative, then p is inside the unit cube
        vec3 d = abs(p) - vec3(1.0, 1.0, 1.0);
        
        // Assuming p is inside the cube, how far is it from the surface?
        // Result will be negative or zero.
        float insideDistance = min(max(d.x, max(d.y, d.z)), 0.0);
        
        // Assuming p is outside the cube, how far is it from the surface?
        // Result will be positive or zero.
        float outsideDistance = length(max(d, 0.0));
        
        return insideDistance + outsideDistance;
    }

    float sphereSDF(vec3 pos) {
        return length(pos) - 1.0;
    }

    float cylinderSDF(vec3 p, float h, float r) {
    // How far inside or outside the cylinder the point is, radially
    float inOutRadius = length(p.xy) - r;
    
    // How far inside or outside the cylinder is, axially aligned with the cylinder
    float inOutHeight = abs(p.z) - h/2.0;
    
    // Assuming p is inside the cylinder, how far is it from the surface?
    // Result will be negative or zero.
    float insideDistance = min(max(inOutRadius, inOutHeight), 0.0);

    // Assuming p is outside the cylinder, how far is it from the surface?
    // Result will be positive or zero.
    float outsideDistance = length(max(vec2(inOutRadius, inOutHeight), 0.0));
    
    return insideDistance + outsideDistance;
}

float sdTriPrism( vec3 p )
{
    vec3 q = abs(p);
    return max(q.z-.5,max(q.x*0.866025+p.y*0.5,-p.y)-.9*0.5);
}

  float sceneSDF(vec3 samplePoint) {
     samplePoint = rotateY(time / 2.0) * samplePoint;
     samplePoint = rotateX(time/2.0) * samplePoint;
    
    float cylinderRadius = 0.4 + (1.0 - 0.4)/ 2.0;
    float sphereDist = sphereSDF(samplePoint);
    float cylinder1 = cylinderSDF(samplePoint, 2.0, cylinderRadius);
    float cubeDist = cubeSDF(samplePoint);
    return differenceSDF(cubeDist, sdTriPrism(samplePoint));

}

    float shortDistFunct(vec3 cam, vec3 dir, float start, float end) {
        float step = start;

        for(int i = 0; i < MAX_STEPS; i++) {
            float dist = sphereSDF(cam + step * dir);
            if(dist < EPSILON) {
                return step;
            }

            step += dist;
            if(step >= end) {
                return end;
            }
        }

        return end;
    }

    vec3 rayDirection(float fieldOfView, vec2 size, vec2 fragCoord) {
        vec2 xy = fragCoord;
        float z = size.y / tan(radians(fieldOfView) / 2.0);
        return normalize(vec3(xy, -z));
    }

    /**
 * Using the gradient of the SDF, estimate the normal on the surface at point p.
 */
vec3 estimateNormal(vec3 p) {
    return normalize(vec3(
        sceneSDF(vec3(p.x + EPSILON, p.y, p.z)) - sceneSDF(vec3(p.x - EPSILON, p.y, p.z)),
        sceneSDF(vec3(p.x, p.y + EPSILON, p.z)) - sceneSDF(vec3(p.x, p.y - EPSILON, p.z)),
        sceneSDF(vec3(p.x, p.y, p.z  + EPSILON)) - sceneSDF(vec3(p.x, p.y, p.z - EPSILON))
    ));
}

/**
 * Lighting contribution of a single point light source via Phong illumination.
 * 
 * The vec3 returned is the RGB color of the light's contribution.
 *
 * k_a: Ambient color
 * k_d: Diffuse color
 * k_s: Specular color
 * alpha: Shininess coefficient
 * p: position of point being lit
 * eye: the position of the camera
 * lightPos: the position of the light
 * lightIntensity: color/intensity of the light
 *
 * See https://en.wikipedia.org/wiki/Phong_reflection_model#Description
 */
vec3 phongContribForLight(vec3 k_d, vec3 k_s, float alpha, vec3 p, vec3 eye,
                          vec3 lightPos, vec3 lightIntensity) {
    vec3 N = estimateNormal(p);
    vec3 L = normalize(lightPos - p);
    vec3 V = normalize(eye - p);
    vec3 R = normalize(reflect(-L, N));
    
    float dotLN = dot(L, N);
    float dotRV = dot(R, V);
    
    if (dotLN < 0.0) {
        // Light not visible from this point on the surface
        return vec3(0.0, 0.0, 0.0);
    } 
    
    if (dotRV < 0.0) {
        // Light reflection in opposite direction as viewer, apply only diffuse
        // component
        return lightIntensity * (k_d * dotLN);
    }
    return lightIntensity * (k_d * dotLN + k_s * pow(dotRV, alpha));
}

/**
 * Lighting via Phong illumination.
 * 
 * The vec3 returned is the RGB color of that point after lighting is applied.
 * k_a: Ambient color
 * k_d: Diffuse color
 * k_s: Specular color
 * alpha: Shininess coefficient
 * p: position of point being lit
 * eye: the position of the camera
 *
 * See https://en.wikipedia.org/wiki/Phong_reflection_model#Description
 */
vec3 phongIllumination(vec3 k_a, vec3 k_d, vec3 k_s, float alpha, vec3 p, vec3 eye) {
    const vec3 ambientLight = 0.5 * vec3(1.0, 1.0, 1.0);
    vec3 color = ambientLight * k_a;
    
    vec3 light1Pos = vec3(4.0 * sin(lightTime),
                          2.0,
                          4.0 * cos(lightTime));
    vec3 light1Intensity = vec3(0.4, 0.9, 0.4);
    
    color += phongContribForLight(k_d, k_s, alpha, p, eye,
                                  light1Pos,
                                  light1Intensity);
    
    vec3 light2Pos = vec3(2.0 * tan(0.37 * lightTime),
                          2.0 * cos(0.37 * lightTime),
                          2.0);
    vec3 light2Intensity = vec3(0.4, 0.4, 0.4);
    
    color += phongContribForLight(k_d, k_s, alpha, p, eye,
                                  light2Pos,
                                  light2Intensity);    
    return color;
}


    void main() {

        vec3 cam = vec3(0.0,0.0,5.0);
        vec3 dir = rayDirection(50.0, resolution, v_pos.xy);
        float dist = shortDistFunct(cam, dir, START, END);

        if(dist > END - EPSILON) {
            gl_FragColor = vec4(0,0,0,.5);
            return;
        }

         vec3 p = cam + dist * dir;
    
    vec3 K_a = vec3(0.5, 0.2, 0.8);
    vec3 K_d = vec3(0.7, 0.2, 0.2);
    vec3 K_s = vec3(1.0, 1.0, 1.0);
    float shininess = 10.0;
    
    vec3 color = phongIllumination(K_a, K_d, K_s, shininess, p, cam);
    
    gl_FragColor = vec4(color, 1.0);
    }
`;
