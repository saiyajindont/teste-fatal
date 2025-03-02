import streamlit as st
import numpy as np
import folium
from folium import Icon, Popup
import osmnx as ox
import networkx as nx
from ortools.constraint_solver import routing_enums_pb2, pywrapcp
from streamlit_folium import st_folium
import os

st.title("Otimizador de Rotas - Pastarosa")
st.sidebar.header("Configurações")

ox.settings.use_cache = True

clientes_base = np.array([
    [-16.6877409, -49.3142917],
    [-16.8092055, -49.2652208],
    [-16.3295005, -48.959735],
    [-16.7000802, -49.3018883],
    [-16.7093551, -49.2972252],
    [-16.686924, -49.2659508],
    [-16.7038435, -49.2712615],
    [-16.6747424, -49.2662128],
    [-16.6991046, -49.2528195],
    [-16.7105064, -49.3001836],
    [-16.6734876, -49.2690427],
    [-16.6387811, -49.2451553],
    [-16.6666242, -49.2618588],
    [-16.7117602, -49.3046789],
    [-16.6388496, -49.2452160],
    [-16.7333320, -49.2352096],
    [-16.6708294, -49.3018824],
    [-16.6799366, -49.2654854],
    [-16.672835, -49.2540433],
])

num_clientes = st.sidebar.slider("Clientes por rota", 5, 7, 6)
clientes = clientes_base[:num_clientes]
st.sidebar.write(f"{len(clientes)} clientes carregados!")

custo_por_km = st.sidebar.number_input("Custo estimado por km (R$)", min_value=0.0, value=0.51)

# 📌 📦 **Carregar grafo OSM de cache se disponível**
grafo_path = "goias_graph.graphml"

if os.path.exists(grafo_path):
    grafo = ox.load_graphml(grafo_path)
else:
    # 📍 Obtendo coordenadas máximas e mínimas para baixar um grafo menor
    north = clientes[:, 0].max() + 0.1
    south = clientes[:, 0].min() - 0.1
    east = clientes[:, 1].max() + 0.1
    west = clientes[:, 1].min() - 0.1

    grafo = ox.graph_from_bbox(north, south, east, west, network_type="drive")

    # 🛠️ Salvar o grafo localmente para evitar downloads futuros
    ox.save_graphml(grafo, grafo_path)

if "crs" in grafo.graph and grafo.graph["crs"] is not None:
    grafo = ox.project_graph(grafo)

# 📍 Encontrar nós mais próximos
nodos_clientes = [ox.nearest_nodes(grafo, lon, lat, method="balltree") for lat, lon in clientes]

# 🔥 **Calcular matriz de distâncias com cache**
@st.cache_data
def calcular_matriz(nodos):
    num_pontos = len(nodos)
    matriz = np.full((num_pontos, num_pontos), float('inf'))
    
    for i in range(num_pontos):
        for j in range(i + 1, num_pontos):
            try:
                dist = nx.shortest_path_length(grafo, nodos[i], nodos[j], weight="length") / 1000  # km
                matriz[i][j] = matriz[j][i] = dist
            except nx.NetworkXNoPath:
                pass
    return matriz

dist_matrix = calcular_matriz(nodos_clientes)

# 🚀 Resolver TSP
def solve_tsp():
    num_pontos = len(nodos_clientes)
    
    if num_pontos < 2:
        return list(range(num_pontos)), 0

    manager = pywrapcp.RoutingIndexManager(num_pontos, 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        return int(dist_matrix[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)] * 1000)

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.seconds = 10  # Reduzido para melhorar desempenho

    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        route = []
        index = routing.Start(0)
        total_distance = 0
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            total_distance += dist_matrix[manager.IndexToNode(previous_index)][manager.IndexToNode(index)]
        return route, total_distance

    return list(range(num_pontos)), float('inf')

rota, distancia_total = solve_tsp()
custo_estimado = distancia_total * custo_por_km

st.dataframe({
    "Cluster": ["Rota 1"],
    "Melhor Rota": [rota],
    "Distância Total (km)": [distancia_total],
    "Custo Estimado (R$)": [custo_estimado]
})

# 🗺️ **Mapa**
st.subheader("🗺️ Visualização das Rotas")
mapa = folium.Map(location=[clientes[:, 0].mean(), clientes[:, 1].mean()], zoom_start=7)

# Adicionar clientes como marcadores
for i, coord in enumerate(clientes):
    folium.Marker(coord, icon=Icon(icon="user", prefix="fa", color="red"), popup=f"Cliente {i+1}").add_to(mapa)

# Adicionar rotas com popups de distância e custo
for i in range(len(rota) - 1):
    nodo1, nodo2 = nodos_clientes[rota[i]], nodos_clientes[rota[i+1]]
    try:
        rota_osm = nx.shortest_path(grafo, nodo1, nodo2, weight="length")
        rota_coords = [(grafo.nodes[n]["y"], grafo.nodes[n]["x"]) for n in rota_osm]
        
        # 🔥 Distância e custo entre os pontos
        distancia = dist_matrix[rota[i]][rota[i+1]]
        custo = distancia * custo_por_km
        
        # 🔥 Criar linha com popup de informações
        linha = folium.PolyLine(
            rota_coords, 
            color="blue", 
            weight=3, 
            opacity=0.8, 
            tooltip=f"Distância: {distancia:.2f} km | Custo: R$ {custo:.2f}"
        )
        mapa.add_child(linha)

    except nx.NetworkXNoPath:
        st.warning(f"⚠️ Sem caminho entre Cliente {i+1} e Cliente {i+2}")

st_folium(mapa, width=1000, height=700)
st.markdown("""Sobre o Otimizador de Rotas:
Esse software utiliza o OpenStreetMap e algoritmos de otimização para encontrar a melhor rota entre diferentes pontos. Ele considera a menor distância possível e os custos associados ao trajeto. 
            Além disso, utiliza aprendizado de máquina para retornar os melhores resultados possíveis.
            Desenvolvido por Wagner Tiburcio.     
""")