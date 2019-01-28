#version 450 core

uniform vec2 iResolution;
uniform mat4 iCamera;
uniform vec4 iMouse;
uniform float iTime;

const vec3 camera_from = iCamera[3].xyz;
const vec3 camera_dir = -iCamera[2].xyz;
const vec3 camera_right = iCamera[0].xyz;
const vec3 camera_up = iCamera[1].xyz;

const int MAX_MARCHING_STEPS = 5;
const float EPSILON = 0.00001f;
const float END = 1000.f;



#define DEBUG 0

float sdBox(vec3 p, vec3 b)
{
	vec3 d = abs(p) - b;
	return length(max(d, 0.0))
			+ min(max(d.x,max(d.y,d.z)),0.0);
}

float sdSphere(vec3 p, float s)
{
	vec3 q = p;
	//q.x = mod(q.x, s*3.0);
	//q.y = mod(q.y, s*3.0);
	//q.z = mod(q.z, s*3.0);
	return length(q - vec3(0.5, 0.0, -2.0)) - s;
}

float sdIntersect(float dA, float dB) {
	return max(dA, dB);
}

float sdScene(vec3 p) {
	//return mod(sdSphere(p - vec3(0.0, 0.0, -2.0), 0.2), 4.0);
	vec3 b = vec3(0.1, 0.1, 0.1);
	//return min(sdSphere(p - vec3(0.0, 0.0, -2.0), 0.2), sdBox(p, b));
	float dSphere = sdSphere(p, 1.0);
	float dBox = sdBox(p, b);
	//return sdIntersect(dSphere, dBox);
	//return dBox;
	return dSphere;
}

vec3 estSurfNormal(vec3 p) {
	return normalize(vec3(
		sdScene(vec3(p.x + EPSILON, p.y, p.z)) - sdScene(vec3(p.x - EPSILON, p.y, p.z)),
		sdScene(vec3(p.x, p.y + EPSILON, p.z)) - sdScene(vec3(p.x, p.y - EPSILON, p.z)),
		sdScene(vec3(p.x, p.y, p.z + EPSILON)) - sdScene(vec3(p.x, p.y, p.z - EPSILON))
	));
}

void GenPerspectiveRay(out vec3 rs, out vec3 rv)
{
	vec3 st = vec3(gl_FragCoord.xy / vec2(iResolution.x), 0.0);
	
	rs = st;
	rs += (-0.5f + st.x) * camera_right;
	rs += (-0.5f + st.y) * camera_up;
	
	rv = normalize(rs - camera_from);
}

void RayMarching(vec3 rs, vec3 rv, out float t) {
	t = 0;
	for (int i = 0; i < MAX_MARCHING_STEPS; i++) {

		float dt = sdScene(rs + t * rv);
		if (dt < EPSILON) {
			return;
		}

		// move along the ray
		// dt will always be positive in this case
		t += dt;

		if (t >= END) {
			t = END+1.0;
			return;
		}
	}
	t = END+1.0;
	return;
}

const int NUM_LIGHTS = 2;
void main()
{
	vec4 color = vec4(0.0, 0.0, 0.0, 1.0);
	vec3 rs, rv;
	GenPerspectiveRay(rs, rv);

	float t;
	RayMarching(rs, rv, t);

	vec3 lights[NUM_LIGHTS] = vec3[](vec3(7.0*cos(iTime), 7.0*sin(iTime), 7.0*cos(iTime)),
							vec3(4.0*cos(iTime*2 + 3.14 / 2.0), 3.0, 4.0*sin(iTime*2 + 3.14 / 2.0))
							);

	vec3 D = vec3(1.0, 0.0, 0.0);
	vec3 A = vec3(0.1, 0.1, 0.1);
	vec3 ambient = D * A;

	if (t < END && t >= 0.0) {
		// calculate lighting
		vec3 Q = rs + t*rv;
		vec3 N = estSurfNormal(Q);

		for (int i = 0; i < 2; i++) {
			//vec3 P = vec3(7.0*cos(iTime), 7.0*sin(iTime), 7.0*cos(iTime)); // Light source location
			//vec3 P = vec3(7.0, 0.0, 7.0);
			vec3 P = lights[i];

			float d = length(P - Q);
			vec3 L = normalize(P - Q); // direction from surface to light source

			vec3 R = 2 * (dot(N, L)) * N - L; // reflected ray
			vec3 V = -rv; // ray to viewer

			vec3 S = vec3(0.0, 1.0, 0.0);

			vec3 specular = S * 0.5 * pow(max(dot(R, V), 0.0), 32) * step(0.0, dot(N, L));
			vec3 diffuse = D * max(dot(N, L), 0.0);

			color.xyz += specular;
			color.xyz += diffuse;
		}

		
		// vec3 C0 = vec3(1.0, 1.0, 1.0); // Color of light
		/*float kc = 1.0;
		float kl = 0.7;
		float kq = 1.8;
		vec3 C = C0 * 1 / (kc + kl * d + kq * d*d);
		*/
		
		
		color.xyz += ambient;// + diffuse + specular;
	}



#if DEBUG
	if (t < END && t >= 0.0) {
		color = vec4(0.0, 1.0, 0.0, 1.0);
	}
#endif

	gl_FragColor = color;
}