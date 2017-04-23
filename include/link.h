
namespace robo{
	class Link{
	public:

		int id;
		Joint joint;
		Frame tip;

		Link(int id, const Joint& joint, const Frame& tip);

		Frame pose(const double& q)const;
		Twist twist(const double& q, const double &dq)const;
	}

}	

		