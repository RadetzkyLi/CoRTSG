class IdHelper:
    @staticmethod
    def agent_id_to_actor_id(agent_id):
        """Convert agent_id to actor_id.
    
        Parameters
        ----------
        agent_id: str
            Customized agent id is of format "{agent_type}_{carla id}", e.g., "cav_1011".

        Returns
        -------
        actor_id: int|None

        """
        if agent_id.startswith("cav"):
            return int(agent_id.split("cav_")[-1])
        elif agent_id.startswith("rsu"):
            return int(agent_id.split("rsu_")[-1])
        else:
            return None

    @staticmethod
    def actor_id_to_agent_id(actor_id, agent_type="cav"):
        """Convert actor_id to agent_id.
        
        Parameters
        ----------
        actor_id: int
            It's supposed to be CARLA.Actor.id.

        agent_type: str
            One of {"cav", "rsu"}. The agent is supposed to be actor that can perceive
            the surrounding environment and share such info with others.

        Returns
        -------
        agent_id: str
            
        """
        if agent_type == 'cav':
            return agent_type + "_" + str(actor_id)
        elif agent_type == 'rsu':
            return agent_type + "_" + str(actor_id)
        else:
            raise ValueError("Unsupported agent type: {0}".format(agent_type))