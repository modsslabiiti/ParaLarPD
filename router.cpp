#include "pch.hpp"
#include "router.hpp"
#include "netlist.hpp"
#include "mst.hpp"
#include "timing.hpp"
#include <cmath>

GridGraph create_grid_graph(int nx, int ny)
{
	GridGraph g;

	g[graph_bundle].nx = nx;
	g[graph_bundle].ny = ny;

	format fmt("%d,%d");
	for (int y = 0; y < ny; ++y) {
		for (int x = 0; x < nx; ++x) {
			fmt % x % y;
			GridGraph::vertex_descriptor v = g.add_vertex(Point(x, y));
			put(vertex_name, g, v, fmt.str());
/*			printf("vertex: %s\n", get(vertex_name, g, v).c_str());*/
		}
	}
	for (int y = 0; y < ny; ++y) {
		for (int x = 0; x < nx; ++x) {
			GridGraph::edge_descriptor e;
			if (x < nx - 1) {
				e = g.add_edge(Point(x, y), Point(x+1, y));
				put(edge_weight, g, e, 1);
			}
			if (y < ny - 1) {
				e = g.add_edge(Point(x, y), Point(x, y+1));
				put(edge_weight, g, e, 1);
			}
		}
	}
	assert(num_vertices(g) == nx*ny && num_edges(g) == 2*(nx-1)*(ny-1)+(nx-1)+(ny-1));
/*	vector_property_map<int, property_map<GridGraph, vertex_index_t>::type> pmap = make_vector_property_map<int>(get(vertex_index, g));*/
/*	printf("num veritices: %d\n", num_vertices(g));*/
/*	printf("vertex:%d\n", vertex(1000,g));*/
/*	pmap[vertex(1000,g )] = 100;*/
/*	printf("size: %d\n")*/
	return g;
}

string current_net_details;

extern int MAX_ITER;
extern int MAX_UTIL;

void Router::parallel_route_tbb_new_netlist(Netlist &netlist, int num_threads)
{
	Mst<GridGraph> mst;
	int max_iter = MAX_ITER;
	boost::timer::nanosecond_type total_routing_time = 0;

	for (const auto &e : grid.edges()) {
		GridGraphEdgeObject &obj = get(edge_object, grid, e);
		obj.mult = 0;
		obj.max_util = MAX_UTIL;
		obj.delay = 1;
		obj.slp=0;
		obj.m=0;
		obj.T1=0;
		obj.T2=0;
	}

	vector<Net> nets;
	nets.reserve(netlist.nets.size());
	int temp = 0;
	for (const auto &net : netlist.nets) {
		Net temp_net;	
		temp_net.index = temp++;
		temp_net.name = net.second->name;
		temp_net.points.emplace_back(net.second->source->port->block->position);

		for (const auto &sink : net.second->sinks) {
			temp_net.points.emplace_back(sink->port->block->position);
		}
/*		printf("%d\n", temp_net.points.size());*/
		nets.emplace_back(std::move(temp_net));
	}

/*	return;*/

	vector<list<Point>> previous_steiners(nets.size());
	int best_iter = -1;
	int best_util = INT_MAX;
	int best_wirelength = -1;
	float best_crit = -1;
	float best_average = -1;
	float ult_crit = FLT_MAX;
	int ult_crit_iter = -1;
	
	//int flag_2=1;
	//int m=0;
	//int t=0;
	//int t_for_min=0;
	//float slp_residual_array[20]={0.0};
	//int max_util_array[20]={0};
	//float slp_residual_min=99999;			// assign maximun float number
	//float change_value=0.0;
	//float change_value_minimum=1.0;
	//int iter=0;
	float rho=0.0001;
	for (int iter = 0; iter < max_iter; ++iter) {
	//while (flag_2) {
		/* best step size so far, 0.01/(iter+1) */
		//float step_size = (float)0.01/(iter+1);
		//iter=iter+1;

		for (const auto &e : grid.edges()) {
			GridGraphEdgeObject &obj = get(edge_object, grid, e);
			obj.util = 0;
			//obj.util_previous = 0;
			assert(obj.delay == 1);
			put(edge_weight, grid, e, obj.delay + obj.mult);
		}
		mst.init(grid);

		vector<list<GridGraph::edge_descriptor>> route_edges(nets.size());

		boost::timer::cpu_timer timer;
		clock_t iter_begin = clock();
		timer.start();

		int total_wirelength = 0;
		const bool nested_zel = true;
		if (iter == 0) {
			if (!nested_zel) {
				int net_num = 0;

				for (const auto &net : nets) {
					mst.parallel_zel_new_reduce(grid, net.points, previous_steiners[net_num]);
					net_num++;
				}
			} else {
				/* previous grain size: 16 */
				tbb::parallel_for(tbb::blocked_range<int>(0, nets.size()), [&mst, &nets, &previous_steiners, this](const tbb::blocked_range<int> &range) -> void
					{
						for (int i = range.begin(); i != range.end(); ++i) {
							if (nets[i].points.size() <= 70) {
								mst.parallel_zel_new_reduce(grid, nets[i].points, previous_steiners[i]);
							}
						}
					});
			}
		}
		//cout<<"AAAAAAA";
		tbb::spin_mutex mutex;
		/* previous grain size: 16 */
		tbb::parallel_for(tbb::blocked_range<int>(0, nets.size()), [&mutex, &route_edges, &total_wirelength, &mst, &nets, &previous_steiners, this](const tbb::blocked_range<int> &range) -> void
			{
				for (int i = range.begin(); i != range.end(); ++i) {
/*					if (nets[i].points.size() <= 30) {*/
/*						mst.parallel_zel_new_reduce(grid, nets[i].points, previous_steiners[i]);*/
/*					}*/
					set<Point> steiners;
					steiners.insert(previous_steiners[i].begin(), previous_steiners[i].end());
					steiners.insert(nets[i].points.begin(), nets[i].points.end());
					route_edges[i] = std::move(mst.kmb(grid, steiners));

/*					set<GridGraph::edge_descriptor> route_edges_check;*/
/*					for (const auto &e : route_edges[i]) {*/
/*						route_edges_check.insert(e);*/
/*					}*/
/*					if (route_edges_check.size() != route_edges[i].size()) {*/
/*						printf("check: %d real: %d\n", route_edges_check.size(), route_edges[i].size());*/
/*					}*/
/*					assert(route_edges_check.size() == route_edges[i].size());*/

					{
						tbb::spin_mutex::scoped_lock lock(mutex);
						total_wirelength += route_edges[i].size();
					}
					for (const auto &e : route_edges[i]) {
						GridGraphEdgeObject &obj = get(edge_object, grid, e);
						{
							tbb::spin_mutex::scoped_lock lock(mutex);
							obj.util += 1;
						}
					}
				}
			}
		);

/***************** calculating f=(c+lambda)*x -w*lambda    *************************
		float f_value=0.0;
		for (const auto &e : grid.edges()) {
			GridGraphEdgeObject &obj = get(edge_object, grid, e);
			f_value=f_value+obj.util*(obj.delay + obj.mult)-MAX_UTIL*obj.mult;
		}
		printf("At iteration number %d, the objective function value is %f\n",iter,f_value);
/***************************************************************************/

/******************step size calculation **********************************/
		float norm_T1T2=0.0;

		for (const auto &e : grid.edges()) {
			GridGraphEdgeObject &obj = get(edge_object, grid, e);
			if (obj.util>obj.max_util){
				obj.m=1;
			}
			else {
				obj.m=0;
			}	
			obj.slp=std::max(0.0f, (float) (obj.util-obj.max_util));
			obj.T1=obj.delay+obj.m*(obj.mult+rho*(obj.slp));
			obj.T2=-obj.slp;
			norm_T1T2=norm_T1T2+sqrt(obj.T1*obj.T1+obj.T2*obj.T2);	
		}
	/*	for (const auto &e : grid.edges()) {
			GridGraphEdgeObject &obj = get(edge_object, grid, e);
			obj.slp=std::max(0.0f, (float) (obj.util-obj.max_util));
			obj.T1=obj.delay+obj.m*(obj.mult+rho*(obj.slp));
		}*/
	/*	for (const auto &e : grid.edges()) {
			GridGraphEdgeObject &obj = get(edge_object, grid, e);
			obj.slp=std::max(0.0f, (float) (obj.util-obj.max_util));
			obj.T2=-obj.slp;
		}*/
	/*	float norm_T1T2=0.0;
		for (const auto &e : grid.edges()) {
			GridGraphEdgeObject &obj = get(edge_object, grid, e);
			norm_T1T2=norm_T1T2+sqrt(obj.T1*obj.T1+obj.T2*obj.T2);
		//printf("for iter=%d m=%d slp=%f T1=%f T2=%f norm of T1;T2=%f\n",iter,obj.m,obj.slp,obj.T1,obj.T2,norm_T1T2);
		}*/
		float step_size = ((float)1.01/(iter+1))/(norm_T1T2);
//		printf("for iter=%d, the step size=%f\n",iter,step_size);
/***********************************************************************************/

		timer.stop();
		clock_t iter_end = clock();
		boost::timer::cpu_times elapsed = timer.elapsed();
		boost::timer::nanosecond_type time = elapsed.user + elapsed.system;
		boost::timer::nanosecond_type real_time = elapsed.wall;

		int total_wirelength_verify = 0;
		int max_util = 0;
		int total_util = 0;
		int total_capacity = 0;
/*		printf("step size: %f\n", step_size);*/
		for (const auto &e : grid.edges()) {
			GridGraphEdgeObject &obj = get(edge_object, grid, e);
/*			assert(obj.max_util == 1);*/
			//obj.mult = std::max(0.0f, obj.mult+step_size*(obj.util-obj.max_util));
			//obj.slp=std::max(0.0f, (float) (obj.util-obj.max_util));
			//printf("slop or gradient : %f\n", obj.slp);
			obj.mult = obj.mult+step_size*(std::max(0.0f, (float) (obj.util-obj.max_util)));;
			//printf("util: %d mult: %f\n", obj.util, obj.mult);
			total_wirelength_verify += obj.util;
			max_util = std::max(max_util, obj.util);
			total_util += obj.util;
		}
/*************** nets size ********************************/
		//printf("Nets size is : %d\n", nets.size());
/**********************************************************/

		assert(total_wirelength_verify == total_wirelength);
		total_capacity = max_util * num_edges(grid);

		const float wire_delay = 0.5*101*2.874e-14 + 551*2.874e-14 + 58e-12;

		/*timing analysis*/
		for (int i = 0; i < nets.size(); i++) {
/*			printf("Net: %s\n", nets[i].name.c_str());*/
/*			for (const auto &p : nets[i].points) {*/
/*				printf("%d,%d ", p.x, p.y);*/
/*			}*/
/*			printf("\n");*/

			Graph<adjacency_list<vecS, vecS, undirectedS, property<vertex_t_arr_t, float>, property<edge_weight_t, float>>, Point> route_tree;
			for (const auto &e : route_edges[i]) {
/*				printf("%d,%d -> %d,%d\n", grid.object(source(e, grid)).x, grid.object(source(e, grid)).y,*/
/*grid.object(target(e, grid)).x, grid.object(target(e, grid)).y);*/
				auto new_e = route_tree.add_edge(grid.object(source(e, grid)), grid.object(target(e, grid)));
				put(edge_weight, route_tree, new_e, wire_delay);
			}
			assert(is_tree(route_tree));
/*			printf("net: %d route_tree: %d\n", nets[i].points.size(), num_vertices(route_tree));*/
/*			assert(nets[i].points.size() == num_vertices(route_tree));*/
			vector<bool> visited(num_vertices(route_tree), false);
			get_delay(route_tree, route_tree.vertex(nets[i].points[0]), 0, visited);
/*		*/
			auto detailed_net = netlist.nets.find(nets[i].name)->second.get();
			assert(detailed_net->name == nets[i].name);
			
			for (const auto sink : detailed_net->sinks) {
				auto e = edge(detailed_net->source->tnode, sink->tnode, netlist.tgraph);
				assert(e.second == true);
/*				printf("%d,%d -> %d,%d: %f\n", detailed_net->source->port->block->position.x, detailed_net->source->port->block->position.y, */
/*						sink->port->block->position.x, sink->port->block->position.y,*/
/*						get(vertex_t_arr, route_tree, route_tree.vertex(sink->port->block->position)));*/
				put(edge_weight, netlist.tgraph, e.first, get(vertex_t_arr, route_tree, route_tree.vertex(sink->port->block->position)));
			}
		}
		float average;
		timing_node crit_tnode;
		float crit = get_critical_path_delay(netlist.tgraph, &average, &crit_tnode);
		timing_node *current = &crit_tnode;

		//printf("---------\n");

		//while (current) {
			//printf("%s %s %d,%d %s[%d] arrival: %g\n", current->pin->port->block->name.c_str(), current->pin->port->block->instance.c_str(), current->pin->port->block->position.x, current->pin->port->block->position.y, current->pin->port->name.c_str(), current->pin->index, current->arrival_time);
			//current = current->prev;
		//}
		//printf("---------\n");


		printf("Iteration: %d Max_util: %d Total_wirelength: %d Runtime: %g, Crit_clb: %g Crit_T: %g Average_clb: %g Average_T: %g\n", iter, max_util, total_wirelength, (double)real_time/1000000000, crit, crit, average, average);

		total_routing_time += real_time;
		if (max_util < best_util) {
			best_util = max_util;
			best_iter = iter;
			best_wirelength = total_wirelength;
			best_crit = crit;
			best_average = average;
		}
		if (crit < ult_crit) {
			ult_crit = crit;
			ult_crit_iter = iter;
		}
	}
	printf("Total_runtime: %g Raw_time: %g Best_iteration: %d Util: %d Wirelength: %d Crit_clb: %g Crit_T: %g Average_clb: %g Average_T: %g\n", (double)total_routing_time/1000000000, (double)total_routing_time, best_iter, best_util, best_wirelength, best_crit, best_crit, best_average, best_average);
	printf("Best crit: %g Best crit iteration: %d\n", ult_crit, ult_crit_iter);

/*	for (const */
}
