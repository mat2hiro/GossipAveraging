# Model of Gossip Averaging
class GossipGraph:
    def __init__(self, n, rad, lambda_val, seed=-1):
        self.n = n
        self.lambda_val = lambda_val
        s_init = st.t.rvs(n, size=n)
        self.x_ave = np.mean(s_init)
        nodes_init_val = {i: {
            's': x,
            'w': 1,
            'm': 0,
            't': 0,
            'h': 0
        } for i, x in enumerate(s_init)}
        if seed >= 0:
            G = nx.random_geometric_graph(n, rad(n), seed=seed)
        else:
            G = nx.random_geometric_graph(n, rad(n))
        nx.set_node_attributes(G, values=nodes_init_val)
        self._G_origin = G
        self.G = G.copy()
        self._hop_min = math.ceil(1/(math.sqrt(2)*rad(n)))
        self._hop_max = math.ceil(math.sqrt(2)/rad(n))+1

    def reset_G(self):
        self.G = self._G_origin.copy()

    def update_G(self, values):
       nx.set_node_attributes(self.G, values=values)

    def get_hop(self):
        return np.random.randint(self._hop_min, self._hop_max)

    def get_Z(self, i):
        i_pos = self.G.nodes[i]['pos']
        I = np.array(i_pos)
        UI = np.array([np.random.uniform(), np.random.uniform()]) - I
        ret = I + 3 * UI / np.linalg.norm(UI)
        return ret.tolist()

    def __get_norm(self, x, y):
       return np.linalg.norm(np.array(x)-np.array(y))

    def get_send_neighbor(self, i, Z):
        closer_nodes = [j for j in self.G.neighbors(i) if self.__get_norm(
            Z, self.G.nodes[j]['pos']) < self.__get_norm(Z, self.G.nodes[i]['pos'])]
        return np.random.choice(closer_nodes) if len(closer_nodes) else -1

    def awake_node(self, i):
        params = self.G.nodes[i]
        Z = self.get_Z(i)
        next_j = self.get_send_neighbor(i, Z)
        msg = self.OneWayMessage(
            i, next_j, params['s'], params['w'], Z, self.get_hop())
        params['t'] += 1
        if msg.H > 1 and next_j >= 0:
            params['w'] /= msg.H
            params['s'] /= msg.H
            params['m'] += 1
            msg.H -= 1
            self.update_G({i: params})
            return msg
        else:
            return None

    def derive_message(self, msg):
        params = self.G.nodes[msg.to_j]
        params['s'] += msg.s
        params['w'] += msg.w
        next_j = self.get_send_neighbor(msg.to_j, msg.Z)
        if msg.H > 1 and next_j >= 0:
            msg.update(next_j, params['s'], params['w'])
            params['s'] /= msg.H
            params['w'] /= msg.H
            params['m'] += 1
            msg.H -= 1
            self.update_G({msg.from_i: params})
            return msg
        else:
            return None

    def oneWayAveraging(self, t_1=0, t_2a=0, t_2b=0):
        self.reset_G()
        t1 = t_1 if t_1 > 0 else self.n
        t2 = t_2a*t1+t_2b if t_2a*t1+t_2b > t1 else self.n*2
        global_time = 0
        msg_queue = deque()
        ret_params = []
        t0_params = dict(self.G.copy().nodes)
        ret_params.append(t0_params)
        while(global_time < t2):
            clock = math.ceil(st.expon.rvs(scale=self.n*self.lambda_val))
            for t in range(clock):
                mqlen = len(msg_queue)
                for m in range(mqlen):
                    new_msg = self.derive_message(msg_queue.popleft())
                    msg_queue.append(new_msg) if new_msg else ''
                global_time += 1

                if(global_time % 500 == 0):
                    print('.', end='')
                if(global_time % n == 0):
                    ret_params.append(dict(self.G.copy().nodes))
                if(global_time == t1):
                    print('*', end='')
                    t1_params = dict(self.G.copy().nodes)
                if(global_time > t2):
                    break

            awk_msg = self.awake_node(np.random.randint(self.n))
            msg_queue.append(awk_msg) if awk_msg else ''
        while(len(msg_queue)):
            new_msg = self.derive_message(msg_queue.popleft())
            msg_queue.append(new_msg) if new_msg else ''

        t2_params = dict(self.G.copy().nodes)
        print('!')

        return t0_params, t1_params, t2_params, ret_params

    def p_awake_node(self, i):
        params = self.G.nodes[i]
        Z = self.get_Z(i)
        next_j = self.get_send_neighbor(i, Z)
        msg = self.PathMessage(
            i, next_j, params['s'], Z, self.get_hop())
        params['t'] += 1
        if msg.H > 1 and next_j >= 0 and params['h'] == 0:
            params['m'] += 1
            params['h'] = 1
            return msg
        else:
            return None

    def p_derive_message(self, msg):
        params = self.G.nodes[msg.to_j]
        next_j = self.get_send_neighbor(msg.to_j, msg.Z)
        if not msg.s_ave is None:
            params['s'] = msg.s_ave
            params['h'] = 0
            msg.back()
        elif msg.H > 2 and next_j >= 0 and params['h']==0:
            msg.go(next_j, params['s'])
            params['h'] = 1
            msg.H -= 1
        elif params['h'] == 1:
            msg.turn()
        else:
            msg.goal(params['s'])
            params['s'] = msg.s_ave
        if msg.to_j >= 0:
            params['m'] += 1
            self.update_G({msg.from_i: params})
            return msg
        else:
            self.update_G({msg.from_i: params})
            return None

    def pathAveraging(self, t_1=0, t_2a=0, t_2b=0):
        self.reset_G()
        t1 = t_1 if t_1 > 0 else self.n
        t2 = t_2a*t1+t_2b if t_2a*t1+t_2b > t1 else self.n*2
        global_time = 0
        msg_queue = deque()
        ret_params = []
        t0_params = dict(self.G.copy().nodes)
        ret_params.append(t0_params)
        while(global_time < t2):
            clock = math.ceil(st.expon.rvs(scale=self.n*self.lambda_val))
            for t in range(clock):
                mqlen = len(msg_queue)
                for m in range(mqlen):
                    new_msg = self.p_derive_message(msg_queue.popleft())
                    msg_queue.append(new_msg) if new_msg else ''
                global_time += 1

                if(global_time % 500 == 0):
                    print('.', end='')
                if(global_time % n == 0):
                    ret_params.append(dict(self.G.copy().nodes))
                if(global_time == t1):
                    print('*', end='')
                    t1_params = dict(self.G.copy().nodes)
                if(global_time > t2):
                    break

            awk_msg = self.p_awake_node(np.random.randint(self.n))
            msg_queue.append(awk_msg) if awk_msg else ''
        while(len(msg_queue)):
            new_msg = self.p_derive_message(msg_queue.popleft())
            msg_queue.append(new_msg) if new_msg else ''

        t2_params = dict(self.G.copy().nodes)
        print('!')

        return t0_params, t1_params, t2_params, ret_params

    class OneWayMessage:
        def __init__(self, from_i, to_j, s_ori, w_ori, Z, H):
            self.from_i = from_i
            self.s = s_ori*(H-1)/H
            self.w = w_ori*(H-1)/H
            self.to_j = to_j
            self.Z = Z
            self.H = H

        def update(self, to_j, s_ori, w_ori):
            self.from_i = self.to_j
            self.s = s_ori*(self.H-1)/self.H
            self.w = w_ori*(self.H-1)/self.H
            self.to_j = to_j

    class PathMessage:
        def __init__(self, from_i, to_j, s_ori, Z, H):
            self.from_i = from_i
            self.to_j = to_j
            self.Z = Z
            self.H = H
            self.path = [from_i, to_j]
            self.s_set = [s_ori]
            self.s_ave = None

        def go(self, to_j, s_ori):
            self.from_i = self.to_j
            self.to_j = to_j
            self.path.append(to_j)
            self.s_set.append(s_ori)

        def goal(self, s_ori):
            self.s_set.append(s_ori)
            self.from_i = self.path.pop()
            self.to_j = self.path.pop() if len(self.path) else -1
            self.s_ave = np.mean(self.s_set)

        def turn(self):
            self.from_i = self.path.pop()
            self.to_j = self.path.pop() if len(self.path) else -1
            self.s_ave = np.mean(self.s_set)

        def back(self):
            self.from_i = self.to_j
            self.to_j = self.path.pop() if len(self.path) else -1

# export gif animation
def convergence_scatter(model, ret_params, tri=False, togif=False, cmap='RdYlBu'):
    frames = []
    if togif:
        fig = plt.figure(figsize=(10,8))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_zlim(-3.5, 3.5)
    for i, param in enumerate(ret_params):
        if not togif:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.set_zlim(-3.5, 3.5)

        x = [v['pos'][0] for v in param.values()]
        y = [v['pos'][1] for v in param.values()]
        z = [v['s'] / v['w'] for v in param.values()]

        va = (np.max(z)-np.min(z)) / 2
        if tri:
            axs = ax.plot_trisurf(x, y, z, cmap=cmap,
                            norm=Normalize(vmin=model.x_ave-va, vmax=model.x_ave+va))
        else:
            axs = ax.scatter(x, y, z, c=z, cmap=cmap,
                       norm=Normalize(vmin=model.x_ave-va, vmax=model.x_ave+va))
        frames.append([axs])
        if not togif:
            plt.show()
    if togif:
        anim = ArtistAnimation(fig, frames, interval=750)
        anim.save('animation{}.gif'.format(model.n), writer='imagemagick')
