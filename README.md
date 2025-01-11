# Cloth Simulation

## Demo
![Demo.gif](https://github.com/goguma1000/Cloth-Simulation/blob/main/Demo.gif?raw=true)
## Code

### Calculate force

particle에 가해지는 힘을 계산하는 loop는 다음과 같다. </br>
~~~c++
void frame( float dt ) {
	const int steps =150;

	for (int i = 0; i<steps; i++)
	{
		vec3 p0 = particles[(count - 1) * count].x;
		vec3 p1 = particles[count * count -1].x;
		for (auto& p : particles) p.clearForce();
		for (auto& p : particles) p.add(p.m * G);
		for (auto& p : particles) p.add(-k_drag*p.v);
		for (auto& s : springs)	  s.addForce();
		for (auto& p : particles) p.update(dt / steps);
		for (auto& p : particles) flooring.resolveCollision(p);
		for (auto& p : particles) sphere.resolveCollision(p,dt/steps);
		if (fix0) {
			particles[(count - 1) * count].x = p0;
			particles[(count - 1) * count].v = {0,0,0};
		}

		if (fix1) {
			particles[count * count -1].x = p1;
			particles[count * count -1].v = { 0,0,0 };
		}
	}
	
}
~~~
우선 particle에 가해진 힘을 초기화 해준 후, 각 particle마다 가해지는 힘을 더해서 알짜힘을 구한다.</br>
그 다음 particle의 위치와 속도를 update한다.</br>
그 후, 주변 물체와 충돌하는지 확인하여 충돌할 경우 particle의 속도를 변화시켜 준다.</br>
만약 위 loop를 한 프레임당 한 번만 돌린다면 오차가 커지고 system이 불안정해진다.</br>
이러한 이유 때문에 한 프레임을 적절한 time step으로 나누어 점진적으로 particle의 속도와 위치를 계산해서 system을 더 안정적으로 만든다.</br>

### Resolve collision

~~~c++
struct Plane {
	vec3 N;
	vec3 p;
	float alpha = 0.6; //반발계수
	float eps = 0.0001f;

	. . .

	void resolveCollision( Particle& particle ) {
		float d = dot(particle.x - p,N);
		if (d < eps) {
			float v = dot(N, particle.v);
			if (v < -eps) { //particle이 오는 경우 평면에 붙을 땐 eps보다 작아야 하므로
				vec3 vn = v * N;
				vec3 vt = particle.v - vn;
				particle.v = vt - alpha * vn;
			}
			else if (v < eps) {
				vec3 vn = v * N;
				vec3 vt = particle.v - vn;
				particle.v = vt;
			}
			particle.x += -d * N; //파고 들어가면 빼줌, 파고 들어가지 않으면 붙임
		}
	}
};
~~~

~~~c++
struct Sphere {
	float r;
	vec3 p;
	vec3 N;
	float alpha = 0.2;
	float mu = 0.5;
	float eps = 0.001f;
	
    . . .
	
    void resolveCollision(Particle& particle, float dt) {
		const vec3 g(0, -980.f, 0);
		N = particle.x - p;
		float d = length(N) - r;
		vec3 N_ = normalize(N);
		if (d < eps) {
			float v = dot(N_, particle.v);
			if (v < -eps) {
				vec3 vn = v * N_;
				vec3 vt = particle.v - vn;
				vt = vt - alpha * vn;
				float Fn = dot(-particle.f, N_); //수직항력
				vt = vt - min(Fn * mu *dt / particle.m, length(vt)) * normalize(vt); //속도변화량으로 마찰력 적용
				particle.v = vt;
			}
			else if (v < eps) { // contact condition
				vec3 vn = v * N_;
				vec3 vt = particle.v - vn;
				particle.v = vt;
			}
			particle.x += -d * N_;
		}
	}
};
~~~

particle과 씬에 있는 물체와 충돌하는지 확인하기 위해 resolveCollision함수를 호출한다.</br>
충돌하는지 확인하기 위해 다음과 같은 공식을 사용한다.</br>
$(X - P) \cdot N < \epsilon$</br>
좌변은 물체와 particle사이의 거리를 나타내며 거리가 $\epsilon$보다 작으면 충돌했다고 판단한다.</br>
이떄 물체의 Normal과 particle의 속도를 내적한 값이 음수면 particle이 물체와 가까워 지는 것을 의미하며, 양수면 particle이 물체와 멀어지는 것을 의미한다.</br>
particle과 물체가 충돌할 때 $-k_{r}V_{n}$을 더하여 표면에서 튕기게 한다.</br>
Sphere같은 경우는 particle과 물체가 충돌했을 때 마찰력을 추가하였다</br>
마찰력 공식은 $F = \mu N$이며 이때 N은 수직항력이다.</br>
수직항력은 particle의 알짜힘과 표면의 Normal을 내적하여 구한다.</br>

### Cloth
Cloth는 Particle과 spring으로 구성되어있다.</br>
Particle과 spring의 구조는 다음과 같다.</br>

~~~c++
struct Particle {
	vec3 x;
	vec3 v;
	vec3 f;
	float m;
	Particle( float mass, const vec3& position, const vec3& velocity=vec3(0) )
		: x( position ), v( velocity ), m( mass ) {}
	void clearForce() {
		f = vec3(0);
	}
	void add( const vec3& force ) {
		f+=force;
	}
	void update( float deltaT ) {
		x += v * deltaT;
		v += f / m * deltaT;
	}
	void draw() {
		drawSphere( x, 1 );
	}
};
~~~

~~~c++
struct Spring {
	Particle& a;
	Particle& b;
	float restLength; // 힘 안받았을 때 길이
	float k = 35.f;
	float kd = 0.1f;
	Spring( Particle& x, Particle& y ) : a(x), b(y), restLength( length(x.x-y.x)) {
	}
	void addForce() {
		//Damped Spring
		vec3 dx = a.x - b.x;
		vec3 dx_ = normalize(dx);
		vec3 f = -1 * (k * (length(dx) - restLength) + kd * dot(a.v-b.v,dx_)) * dx_;
		a.add(f);
		b.add(-f);
	}
	void draw() {
		drawCylinder( a.x, b.x, 0.4, glm::vec4(0,1,.4,1) );
	}
};
~~~

Particle은 위치, 속도, 힘을 변수로 가진다.</br>
Spring은 서로 연결되어 있는 particle 2개와 용수철 상수, damping상수를 갖고있다.</br>
addForce함수는 spring에 연결된 particle에 힘을 계산해서 더해주는 함수이다.</br>
damped spring모델을 적용하였으며 공식은 다음과 같다.</br>
<span style="font-size:150%">
$-[ k_{s}(\left\vert \Delta x \right\vert - r) + k_{d}(\frac{\Delta v \cdot \Delta x}{\left\vert \Delta x \right\vert})]\frac{\Delta x}{\left\vert \Delta x \right\vert}$</br>
<span>


~~~c++
for (int y= 0; y < count; y++) {
		for (int x = 0; x < count; x++) {
			particles.push_back(Particle(0.0008f, { x*2 -5.0f ,y * 2 + 62 ,randf() * 0.1f}));
		}
	}
	for (int y = 0; y < count; y++) {
		for (int x = 0; x < count-1; x++) {
			springs.push_back(Spring(particles[y * count + x], particles[y * count + (x + 1)]));
		}
	}
	for (int y = 0; y < count -1; y++) {
		for (int x = 0; x < count; x++) {
			springs.push_back(Spring(particles[y * count + x], particles[(y + 1) * count + x]));
		}
	}
	for (int y = 0; y < count - 1; y++) {
		for (int x = 0; x < count - 1; x++) {
			springs.push_back(Spring(particles[y * count + x], particles[(y + 1) * count + (x + 1)]));
		}
	}
	for (int y = 0; y < count - 1; y++) {
		for (int x = 0; x < count - 1; x++) {
			springs.push_back(Spring(particles[y * count + (x + 1)], particles[(y + 1) * count + x]));
		}
	}
~~~
Cloth는 위와 같이 particle에 카메라의 view 방향으로 약간의 offset을 두고 spring으로 particle들을 연결하여 생성하였다.</br>

