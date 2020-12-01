#pragma once
#include <vector>
#include "glm/glm.hpp"

class Vehicle;

// \brief: ���һ�������Ľṹ
struct Lane
{
	Lane* left = nullptr;		//��೵��
	Lane* right = nullptr;	//�Ҳ೵��
	Lane* previous = nullptr;	//ǰ�򳵵�
	Lane* next = nullptr;		//���򳵵�
	std::vector<glm::dvec3> points_;		//�������ϵ�ȫ�������

	std::vector<Vehicle*> vehicles_;	//��ǰ�ó����ϵĳ���
};