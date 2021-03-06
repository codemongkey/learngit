// const.cpp: 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include<string>
#include<iostream>//input output stream
#include<vector>
#include<algorithm>
#include<string.h>
#include<assert.h>
#include<vector>
#include<deque>
#include<list>
#include<set>
#include<iomanip>
#include<fstream>
#include<exception>
using namespace std;//全局命名空间
const double Pi = 3.14159265;
//trlen函数的意思是测试字符串的字符长度，不含字符串结束标志的。
//sizeof是个运算符，它的结果是字符串在内存中的所占字节大小，它要把\0算进去的。
class Human
{
private:
	string name;
	int age;
public:
	//Human()//默认构造函数 构造函数可重载
	//{
	//	age = 0;
	//	cout << "Default constrctor: name and age not set" << endl;
	//}
	//Human(string humansname = "Adam", int humansAge = 25)//这也是默认构造函数
	//{
	//	name = humansname;
	//	age = humansAge;
	//	cout << "Overload constructor creates" << name;
	//	cout << "of age" << age << endl;
	//}
	Human(string humansname = "Adam", int humansAge = 25)
		  :name(humansname),age(humansAge)//初始化链表 无需在赋值                          
	{
		cout << "Overload constructor creates" << name;
		cout << " of age " << age <<" of name " << humansname<<endl;
	}
	explicit Human(int humansAge) :age(humansAge) {};//explicit用于静止隐式转换；
	//Human(string humansname, int humansAge = 25)//这不是默认构造函数
	//{
	//	name = humansname;
	//	age = humansAge;
	//	cout << "Overload constructor creates" << name;
	//	cout << "of age" << age << endl;
	//}
	void IntroduceSelf()
	{
		cout << "I am " + name << " and am ";
		cout << age << " years old" << endl;
	}
};
class MyString
{
private:
	char* buffer;
public:
	//MyString(const char* initString)
	//{
	//	if (initString != NULL)
	//	{
	//		buffer = new char[strlen(initString) + 1];
	//		strcpy(buffer, initString);
	//	}
	//	else
	//		buffer = NULL;
	//}
	/*复制构造函数必须按引用传递参数 否则将循环调用复制函数  
	如果没提供 编译器会自动提供 会导致浅复制*/
	MyString(const MyString& copySource)
	{
		buffer = NULL;
		if (copySource.buffer != NULL)
		{
			buffer = new char[strlen(copySource.buffer) + 1];
		}
	}
};
//Human::Human()//这是默认构造函数
//{
//	name = humansname;
//	age = humansAge;
//	cout << "Overload constructor creates" << name;
//	cout << "of age" << age << endl;
//}
//#pragma warning(disable:4996)
class President
{

public:
	int name;	    
		President()
		{
			this->name = 1;
		};	
	static President& GetInstance(President* sname)//静态成员实例 不能通过对象来调用
	{
	    President onlyInstance;//static 不加stalic 他也是静态的
		sname->name = 1;
		return onlyInstance;
	}
    private:
	
	President(const President&);//私有复制函数 如使用= 或赋予函数参数
	President& operator=(const President&);//私有赋值运算符

};
class MonsterDB
{
private:
	~MonsterDB()//实现不能在栈中实例化 栈空间通常有限；
	{

	};
public:
	static void DestoryInstance(MonsterDB* pInstance)
	{
		delete pInstance;
	};
	void DoSomething() {};
};
void presentPresident(President)
{

}
//int MAX(int a, int b)
//{
//	if (a > b)
//		return a;
//	else
//		return
//		b;
//}
int MAX(double a, double b)
{
	if (a > b)
		return a;
	else
		return
		b;
}
 class  someclass
{
 private:
	 int name;
 public :
	  void DisplayMember() const//const int name
	 {
		// cout << name;
	 }  ;//
};
void displayalldata(const someclass& object)
{
	object.DisplayMember();
}
template <typename objType>//模板
const objType& GetMax(const objType& value1, const objType& value2)//模板函数
{
	if (value1 > value2)
		return value1;
	else
		return value2;
}
template <typename T>
void Display(const T& container1)
{
	for (auto element = container1.cbegin(); element != container1.cend(); element++)
		cout << *element << ' ' ;
	cout << endl;
}
bool sort_bescending(const int&lhs, const int& rhs)
{
	return(lhs < rhs);
}
//template <typename T>
struct stsample
{
	string name;
	string phone;
	string display;
	stsample(const string nameInit, const string& phoneInit)
	{
		name = nameInit;
		phone = phoneInit;
		display = (name + ":" + phone);
	}
	bool operator == (const stsample& itemToCompare) const
	{
		return (itemToCompare.phone == this->phone);//this 为指针 不能用. 要用->
	}
	bool operator < (const stsample& itemToCompare)const
	{
		return(this->name < itemToCompare.name);
	}
	//bool operator()(const T& lhs, const T& rhs)
	//{
	//	return(lhs > rhs);
	//}
	operator const char*() const//不明其意义 但通过自定义 使cout通过
	{
		return display.c_str();
	}
};
struct compare_sts
{
	bool operator()(const stsample& first, const stsample second) const
	{
		if (first.name < second.name)
			return true;
		else
			return false;
    }
};
double Divide(double dividen, double divisor)
{
	if (divisor == 0)
		throw "Dividing by 0 is a crime";
	return (dividen / divisor);
}

class GamePlayer
{
private:
	static const int NumTurns =10 ;//声明式 只要不取地址 你可以申明他们而无需定义Turns	
	enum { Num = 5 };
	int ab = *(&NumTurns);
	int a[Num];
	static int num1;
  public:
	  GamePlayer() {
	  };
	void show()
	{
		cout<<num1<<endl;
		cout<< &NumTurns <<endl;
	}
};
int GamePlayer::num1;
//const int GamePlayer::NumTurns;//定义式 可取地址
void out(char a)
{
	a = 'g';
	cout << a<< endl;
}
class TextBlook
{
private:
	std::string text;
public:
	TextBlook(const char * st)
	{
		text = st;
	}
	const char& operator[](std::size_t position) const
	{
		return text[position];
	};
	char& operator[](std::size_t position) 
	{
		return text[position];
	};
};
bool isPalindrome(long x)
{
	//optimizations for cust time
	if (x<0) return false;
	if (x<10) return true;
	if (x % 10 == 0) return false;
	if (x<100 && x % 11 == 0) return true;
	if (x<1000 && ((x / 100) * 10 + x % 10) % 11 == 0) return true;

	//actual logic
	int v = 0;
	v = x % 10;
	x = x / 10;
	while (x - v>0)
	{
		v = v * 10 + x % 10;
		x /= 10;
	}
	if (v>x) { v /= 10; }
	return v == x ? true : false;
}
bool IsPrime(int n)
{
	if (n == 1)
		return false;
	for (int i = 2; i<n; ++i)
	{
		if (n%i == 0)
			return false;
	}
	return true;
}
char a[5];
int main()
{  
	//int a = '\01';
	//cout << a << endl;
	//cout << "123\0 1";
	//cout << "123\0";
	const char* a;
	string yournum;
	int ab;
	cin >> ab;
	if (cin.fail())
	{
		cin.clear();
		//cin.sync();
		//cin.get();
		//cin.ignore();
		cin.ignore(1000, '\n');//
		//cin.ignore(1000, '\n');
	}
	else
		cin.ignore();
	//cin.ignore(1024, '/n');//清除当前行
	getline(cin, yournum, '#');
	cout << yournum << endl;
	size_t charcounter = 0;
	while (yournum[charcounter++]);
	charcounter -= 2;
	cout << "输入长度" << charcounter << endl;
	cout << "第一个字符" << yournum[0] << endl;
	//getline(cin, yournum);
	cout << yournum << endl;
	char Again;
	do
	{
		cout << "please input a integer: ";
		getline(cin, yournum,'#');
		//vector <int> intergers;
		size_t charcounter = 0;
		while (yournum[charcounter++]);
		charcounter -= 2;
		cout << "输入长度" << charcounter<<endl;
		cout << "最后字符" << yournum[0]<<endl;
		int i = 0;
		for (i = 0; i < charcounter; ++i, --charcounter)
		{
			if (yournum[i] != yournum[charcounter])
			{
				break;
			}
		}
		if (i < charcounter)
		{
				cout << "No" << endl;
		}
		else
		{
			cout << "Yes" << endl;
		}
		cout << "Again? Y/N ";
		cin >> Again;
		/*cout << endl;*/
		//for (i = 0; i < charcounter; ++i)
		//{
		//}
	} while (Again == 'Y');
	//vector<int>::iterator element = intergers.begin();
	////intergers = getchar();
	//cout << *element<<endl;
	//    char a[100];
	//	cin >> a;
	//	//a[0] = '\0';
	//	//a[1] = '1';
	//	int len = 0;
	//	while (a[len++]);//++放在条件里更加整洁 但是最后要多减去1
	//	len -= 2;
	//	cout << "输入长度" << len;
	//	int i = 0;
	//	for ( i= len + 1; i < 10; i++)
	//	{
	//		if(a[i]=='\0')
	//			cout << 'b';
	//		else break;
	//	}
	//	cout << "空格符长度" << i;
		//cout << "length" << len;
	//	for (int i = 0; i<5; i++)
	//{
	//    cout << a[i];
	//	//cout << '12\012';
	//	if (a[i] == '\0')
	//	{
	//		cout << 'b';
	//	}
	//}
	//	cout << "end" << endl;  //以end标记输出结束位置，输出会有空格
	//	for (int i = 0; i<5; i++)
	//	{
	//		cout << (int)a[i] << ' ';

	//	}
	//char buf[50];
	//char Again = 'Y';
	//int i = 0;
	//do
	//{
	//	for (int i = 0; i < 50; i++)
	//		buf[i] = NULL;
	//	cout << "please input a integer: ";
	//	cin >> buf;
	//	int len = 0;
	//	while (buf[len++]);
	//	len -= 2;		
	//	for ( i = 0; i < len; ++i, --len)
	//	{
	//		if (buf[i] != buf[len])
	//		{
	//			break;
	//		}
	//	}
	//	if (i < len)
	//	{
	//			cout << "No" << endl;
	//	}
	//	else
	//	{
	//		cout << "Yes" << endl;
	//	}
	//	cout << "Again? Y/N ";
	//	cin >> Again;
	//} while (Again == 'Y');
	////cout << s << endl;
	//int n = 0;

	//vector<int> prime;
	//cin >> *prime;
	//cout << *prime;
	//std::string str = "123";
	//int n = atoi(str.c_str());
	//cout << n << endl;
	/*TextBlook tb("hello"); 
	register int i = 0;
	const TextBlook ctb("world");
	cout << tb[0] << endl;
	char x = 'b';
	out(x);
	cout << x;*/
	//long   UserNum = 0;
	//char Again = 'Y';
	//do
	//{
	//	cout << "please input a integer: ";
	//	cin >> UserNum;
	//	switch (isPalindrome(UserNum))
	//	{
	//	case true:
	//	{
	//		cout << "Yes!" << endl;
	//		break;
	//	}
	//	default:
	//		cout << "No!" << endl;
	//	};
	//	cout << "Again? Y/N ";
	//	cin >> Again;

	//}
	//while (Again == 'Y');
	//tb[0] = 'a';
	//ctb[0] = 'a';
	//int i, n, cnt = 0;
	//int a = 0, b = 0, c = 0,d=0;
	//for (i = 10; i < 100; i++)
	//{
	//	n = i * i;
	//	if (n < 1000)
	//	{
	//		a = n % 10;
	//		b = (n / 10) % 10;
	//		c = n / 100;
	//		if (a == b || b == c)
	//		{
	//			cnt++;
	//			cout << n << " " ;
	//		}
	//	}
	//	else
	//	{
	//		a = n % 10;
	//		b = (n / 10) % 10;
	//		c = (n / 100) % 10;
	//		d = n / 1000;
	//		if (a == b || b == c||c==d)
	//		{
	//			cnt++;
	//			cout << n << " " ;
	//		}
	//	}
	//	if (cnt == 10)
	//		cout << endl;
	//}
	//const int ba = 10;
	//GamePlayer x;
	//x.show();
	//cout << *(&ba) << endl;
	//try 
	//{
	//	cout << "good" << endl;
	//	cout << "Result is:" << Divide(23, 0) << endl;
	//}
	//catch (const char* exp)
	//{
	//	cout << "Exception:" << exp << endl;
	//	cout << "sorry,can't continue!" << endl;
	//}
	
	//set<stsample> thad { stsample{ "Jundi", "1889" }, stsample{ "Jun","188" } };//;, compare_stsmulti 对于类 必须提供<和==
	//thad.insert(stsample("Aun","00"));
	//Display(thad);
	////auto ed = find(thad.begin(),thad.end(),stsample("","188"));//线性标准stl的find调用 operator==
	//auto ed = thad.find(stsample("", "188"));//二叉树set的find调用 operator<
	//if (ed != thad.cend())
	//	cout << *ed;
	//else cout << "not found";
	//int a = 10, b = 20;
	////cout;
	//cout << dec << setiosflags(ios_base::dec | ios_base::showbase | ios_base::uppercase);
	//cout << a << endl;
	//cout << b<<ends<<dec<<b;
	//double pi = (double)22.0 / 7;
	//
	//cout << setprecision(1);// << fixed << pi << endl;
	//cout << pi << endl;
	//cout << scientific << pi<<endl;

	//cout << setw(20) << setfill('*');
	//cout << "good that much c++" << endl;
	//cout << "good that much c++" ;

	//char a, b, c;
	//cin >> a >> b >> c;
	//char  charbuff[10] = { 0 };
	//cin.get(charbuff, 9);
	//cout << charbuff;
	//string name;
	//cin >> name;
	//cout << name << endl;
	//getline(cin, name);
	//cout << name << endl;
	//fstream myfile;
	//myfile.open("Hell0File.txt", ios_base::in | ios_base::out | ios_base::trunc);
	//if (myfile.is_open())
	//{
	//	myfile.close();
	//}
	//stsample  sample { "good" };
	//cout << sample<<endl;
	//multiset<int> setsam{0, 2, 2, 2, 3, 1};
	//auto element = setsam.find(2);
	//setsam[3] = 3;//不能通过迭代器改变值
	//cout << *(element--);//迭代器--和-1是不同的哇 也没有-=操作 (setsam.count(2)-1)
	//vector<int> intergers{ 1,2,3,4 };
	//auto bb = find(intergers.begin(),intergers.end(),2);
	//
	//cout << *bb << endl;*bb = 1;
	//cout << *bb;
	//intergers[0] = 2;
	//deque<int> intDeque{ 1,2,3,4 };
	//list<int> listint{ 1,2,3,4 };
	//auto liter = listint.cbegin();
	//listint.reverse();//链表重新排序后 其迭代器仍指向该元素
	//listint.sort(sort_bescending);
	//Display(listint);
	//cout << *liter << endl;
	//intDeque.pop_front();
	//cout << intergers.capacity() << endl;
	//intergers.reserve(5);//增加内存
	//cout << intergers.capacity() << endl;
	//cout << intDeque[0]<<endl;
	////Display<int>(intergers);
	//cout << intergers.capacity() << endl;
	//intergers.reserve(10);
	//cout << intergers.capacity() << endl;
	//cout << intergers[3];
	//cout<<  endl;
	////cout << intergers.at(4)<<endl;//使用at函数 在运行前检查容器大小 若索引超出边界，将引发异常
	//cout << *(intergers.end()-1);
	//string str2("C++14\0initialization"s);//\0只是一个空格
	//cout << str2<<endl;
	//int b = MAX(2, 3);
	//b = GetMax<int>(25, 40);//调用模板函数时可不显示指定类型
	//char* say = NULL;
	//assert(say != NULL);       
	//int v;
	//v = MAX(2.1, 3);
	//int a = 3;c
	//double b = 4.2;
	//a = (int)b;
	//cout << a << " " << b;
	//0表示false否则为true
	/*const*/
	{
	  //const int bNum = 100;
	//int cNum = 10;
	//const int& aNum = cNum;
	//cout << aNum << endl;
	}
	/*continue break*/
	{
	/*for (int i = 0; i < 3; i++)
	{
		cout << i << endl;
		break;
		continue;
		i++;
	}*/
	}
	/*基于范围的for循环*/
	{
		/*int aNum[] = { -1,0,2,2 };
		for (auto arry : aNum)
		{
            if(arry==2)
			cout << arry << endl;
		}*/
	}
	/*带默认值的函数参数*/
	{
		//double Area(double radius, double Pi = 3.14);
		//cout << Area(1,3.1415) << endl;//
		//cout << Area(1) << endl;//pi仍然默认是3.14
	}
	/*函数重载,数组形参 c不支持*/
	{
	//void DisplayArrary(int numbers[], int length);
	//int numbes[] = {-1,0,1,2};
	//DisplayArrary(numbes, 4);
	}
	/*按引用传递参数 c不支持  return 只能返回一个值*/
	{
		//void Area(double radius, double& result);
		//double result = 0;
		//Area(2, result);
		//cout << result;
	}	
	/*lambda函数 使用STL算法*/
	{
		//void DisplayNums(vector<int>& dynArray);
		//vector<int>myNums;//动态数组
		//myNums.push_back(501); Q
		//myNums.push_back(-1);
		//myNums.push_back(25);
		//myNums.push_back(-35);
		//DisplayNums(myNums);

		//sort(myNums.begin(), myNums.end(),[](int Num1, int Num2) {return(Num2 > Num1); });			
		//DisplayNums(myNums);
	}
	/*指针与取地址*/
	{
		//int age = 30;
		//int* pointsToInt = &age;
		//cout << &age << " " << pointsToInt<<" "<<*pointsToInt<<endl;
		//*pointsToInt = 20;
		//cout << age;
		//cout << sizeof(char) << endl;
		//cout << sizeof(int) << endl;
		//cout << sizeof(double) << endl;
		//cout << sizeof(char*) << endl;
		//cout << sizeof(int*) << endl;
		//cout << sizeof(double*) << endl;
	}
	/*const用于指针*/
	{
		//int a = 20;
		//int b = 30;
		//int* const pointa = &a;//指针不可改变 但是指针对应的变量可以
		//*pointa = 21;
		////pointa = &b;是错误的
		//const int* pointb = &a;//指针可改变 但是指针对应的变量不可以
		//pointb = &b;
		////*pointb = 21;
		//const int* const pointc = &a;//都不可改变
		////pointc = &b;
	 //   // *pointc = 21;
	}
	/*将指针传递给函数时，函数就不需返回 直接通过地址修改变量值*/
	{
		//void CalcArea(const double* const ptrPi, const double* const ptrRadius, double* const ptrArea);
		//double radius = 0;
		//cin >> radius;
		//double area = 0;
	 //   CalcArea(&Pi, &radius, &area);
		//cout << area;
	}
	/*数组与指针*/
	{
		//int mynumbers[5];
		//int* pointToNums = mynumbers;
		//*mynumbers = 0;
		//cout << *pointToNums<<" "<<pointToNums[0];
	}
	/*new delete*/
	{
		//void outstring(const char* initstring);
		//outstring("every");
		//int num = 0;
		//cin >> num;
		//int* myNumbers = new int[num];
		//*myNumbers = 10;
		//*(myNumbers+1) = 100;
		//cout << *(myNumbers + 1);
		//char* buffer;//
		//char* buffer2;//
		////buffer = buffer2;
		//buffer = new char[10];
		//*buffer = "1";
	}
	/*程序分配内存失败的异常处理机制try-catch new(nothrow)*/
	{
		//try 
		//{
		//	int* pointToManyNums = new int[0x1fffffff];
		//	delete[] pointToManyNums;
		//}
		//catch (bad_alloc)
		//{
		//	cout << "Memory allocationg failed.Ending program" << endl;
		//	while (1);
		//}
		//int* pointToManyNums = new(nothrow) int [0x1fffffff];//内存分配失败时返回Null
		//if (pointToManyNums)
		//{
		//}
		//else 
		//	cout << "Memory allocationg failed.Ending program" << endl;
	}
	/*引用是变量的别名 但是可以访问内存单元（加const禁止） 入函数即可通过引用参数返回结果*/
	{
	//	void GetSquare(int& number);
	//	int number = 5;
	//	GetSquare(number);
	//	const int& number1 = number;
	//	//number1 = 10; 
	//	cout << number;
	}
	/*类*/
	{
	//Human firstman;
	//Human secondman("adam", 20);
	//Human thirdman("eve");
	//firstman.IntroduceSelf();
	//secondman.IntroduceSelf();
	//thirdman.IntroduceSelf();
		//const char* test= "abde";
		//const char* test1 = "abe";
		//char* get=NULL;
		//char* get1;
		////test = "a";
		//cout << strlen(test) << endl;
	 //   get = new char[strlen(test1)+1];
		//get1 = get;
		//strcpy_s(get, 4,test1);//如果此处给get复制大于他所分配内存的字节 那么在delete时会出错 内部原因未理解
		////strcpy_s(get, 5, test1);//
		//cout << get<<endl;
		//cout << 20 << endl;
		//delete[] get;
		//cout << 20 << endl;
		//get = new char[strlen(test)+1];
		//strcpy_s(get, strlen(test) + 1, test);
		//cout << get << endl;
		//delete[] get;		
	}
	/*私有复制函数*/
	{
		//President one;
		//President two;
		//two = one;
		//presentPresident(one);
		//int a = 2;
		//int b = 0;
		///*引用初始化后，不可以修改。
		//指的不是不能修改它引用的变量的值，而是不能让它在引用其他变量*/
		//int& x=a;
		//a = 3;
		//x = 2;
		////&x = b;
		//cout << x << endl;
		//President& onlyPresident = President::GetInstance(this);//
		//President& secondPresident = President::GetInstance(); //= President::GetInstance();
		//onlyPresident.name = 1;
		//secondPresident.name = 4;
		//cout << onlyPresident.name;
		//MonsterDB* myDB = new MonsterDB();//使用指针定义时必须要初始化 即给出对应地址才能使用
		//myDB->DoSomething();
		////delete myDB;会调用析构函数 无法调用私有析构函数
		//MonsterDB::DestoryInstance(myDB);
		//Human onePerson = 10;一般要禁止隐式转换
	}
	system("pause");
    return 0;
    
}
double Area(double radius)
{
	return 2 * Pi*radius;
}
double Area(double radius,double height)
{
	return 2 * Pi*radius;
}
void DisplayArrary(int numbers[],int length)
{
	for (int index = 0; index < length; index++)
		cout << numbers[index]<<" ";
	cout << endl;//输出endl即跳行
}
void DisplayArrary(char characters[], int length)
{
	for (int index = 0; index < length; index++)
		cout << characters[index]<<" ";
	cout << endl;
}
void Area(double radius, double& result)
{
	result = 2 * Pi*radius;
}
inline double GetPi()//内联函数 请求编译器将函数内的代码直接放到调用他的地方执行 提高代码执行速度 inline函数代码要非常简单 防止代码膨胀
{
	return 3.14159;
}
auto aGetPi()//auto是没有必要使用的操作且毫无意义
{
	double Pi = 3.14159;
	return Pi;
}
void DisplayNums(vector<int>& dynArray)//形参是动态数组的地址
{
	for_each(dynArray.begin(), dynArray.end(), \
		[](int element) {cout << element << " "; });
	cout << endl;
}
void CalcArea(const double* const ptrPi, const double* const ptrRadius, double* const ptrArea)
{
	if (ptrPi&&ptrRadius&&ptrArea)//检查指针是否为Null
		*ptrArea = (*ptrPi) * (*ptrRadius) * (*ptrRadius);
}
void outstring(const char* initstring)
{
	cout << *(initstring+1)<<" "<<strlen(initstring)<<" "<< initstring<<endl;
	string mynae = initstring;
	const char* a;
	int* b;
	a = "e";
	string wh = "abd";
	char* x;
	x = new char[4];
	//strcpy(x, a);
	cout << x<<endl;
	//int* c = a;
	//char* d = a;
	//a = initstring;
	//char* mmynae = initstring;
	cout << mynae;
}
void GetSquare(int& number)
{
	number *= number;
}