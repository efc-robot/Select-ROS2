// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * @file RTPSWriter.cpp
 *
 */

#include <fastdds/rtps/writer/RTPSWriter.h>

#include <fastdds/dds/log/Log.hpp>

#include <fastdds/rtps/history/WriterHistory.h>
#include <fastdds/rtps/messages/RTPSMessageCreator.h>

#include <rtps/history/BasicPayloadPool.hpp>
#include <rtps/history/CacheChangePool.h>
#include <rtps/flowcontrol/FlowController.h>
#include <rtps/participant/RTPSParticipantImpl.h>
#include <fstream>
#include <mutex>
#include <vector>
#include <algorithm>
#include <chrono>
#include <ctime>
#include <sys/time.h>

struct machineID
{
    int ID1;
    int ID2;
    int ID3;
    int ID4;
};

struct judge_table
{
    int ModuleID;
    int PriorityID;
    machineID MachineID;
    double Latency;
    double CPUCost;
    double GPUCost;
    bool Running = false;
};

struct source_table
{
    double CPUIdle;
    double GPUIdle;
};
source_table read_source_from_txt(machineID mID)
{
    source_table table;
    string txt_file;
    txt_file = "/home/nics/my_hub/source/";
    txt_file += to_string(mID.ID1);
    txt_file += "-";
    txt_file += to_string(mID.ID2);
    txt_file += "-";
    txt_file += to_string(mID.ID3);
    txt_file += "-";
    txt_file += to_string(mID.ID4);
    txt_file += ".txt";
    string str;
    ifstream ifs;

    ifs.open(txt_file, ios::in);
    if (!ifs.is_open())
    {
        std::cout << "open file error" << endl;
        return table;
    }
    // 一直读入
    while (getline(ifs, str))
    {
        // 读取使用空格分割的字符串
        int i = 0;
        while (str[i] != ' ')
        {
            i++;
        }
        table.CPUIdle = std::stod(str.substr(0, i));
        table.GPUIdle = std::stod(str.substr(i + 1, str.length() - i - 1));
    }
    ifs.close();

    return table;
}

bool compare(judge_table a, judge_table b)
{
    return a.PriorityID < b.PriorityID;
}

void write_txt(std::string txt_file, std::string data)
{
    std::ofstream ofs;
    ofs.open(txt_file, std::ios::out);
    ofs << data << std::endl;
    ofs.close();
}

std::string read_txt(std::string txt_file)
{
    std::ifstream ifs;
    ifs.open(txt_file, std::ios::in);
    if (!ifs.is_open())
    {
        std::cout << "文件打开失败" << std::endl;
        return "error";
    }
    std::string line;
    std::string data;
    while (std::getline(ifs, line))
    {
        data = line.c_str();
        std::cout << data << std::endl;
    }
    ifs.close();
    return data;
}

void init_source(machineID mID, double cpu, double gpu)
{
    // get mID info
    string txt_file = "/home/nics/my_hub/source/";
    txt_file += std::to_string(mID.ID1);
    txt_file += "-";
    txt_file += std::to_string(mID.ID2);
    txt_file += "-";
    txt_file += std::to_string(mID.ID3);
    txt_file += "-";
    txt_file += std::to_string(mID.ID4);
    txt_file += ".txt";
    // init source info
    // 获得两个0.5~0.9的随机数
    source_table table = read_source_from_txt(mID);
    table.CPUIdle += cpu;
    table.GPUIdle += gpu;
    string data = std::to_string(table.CPUIdle) + " " + std::to_string(table.GPUIdle);
    write_txt(txt_file, data);
}

void modify_source(judge_table table1)
{
    // 修改已消耗的资源
    string txt_file = "/home/nics/my_hub/source/";
    txt_file += std::to_string(table1.MachineID.ID1);
    txt_file += "-";
    txt_file += std::to_string(table1.MachineID.ID2);
    txt_file += "-";
    txt_file += std::to_string(table1.MachineID.ID3);
    txt_file += "-";
    txt_file += std::to_string(table1.MachineID.ID4);
    txt_file += ".txt";
    // init source info
    string str;
    ifstream ifs;
    double cpuidle, gpuidle;
    ifs.open(txt_file, ios::in);
    if (!ifs.is_open())
    {
        cout << "open file error" << std::endl;
    }
    // 一直读入

    while (getline(ifs, str))
    {
        int i = 0;
        while (str[i] != ' ')
        {
            i++;
        }
        cpuidle = std::stod(str.substr(0, i));
        gpuidle = std::stod(str.substr(i + 1, str.length() - i - 1));
    }
    ifs.close();
    double cpucost = table1.CPUCost;
    double gpucost = table1.GPUCost;
    string data = std::to_string(cpuidle - cpucost) + " " + std::to_string(gpuidle - gpucost);
    write_txt(txt_file, data);

    // 记录已执行
    string txt_file2 = "/home/nics/my_hub/judge/";
    txt_file2 += std::to_string(table1.ModuleID);
    txt_file2 += "-";
    txt_file2 += std::to_string(table1.PriorityID);
    txt_file2 += ".txt";
    string data2 = "1";
    write_txt(txt_file2, data2);
}

bool GetRunningInfo(int Module, int Priority, machineID mID, double cpu, double gpu)
{
    // get mID info
    string txt_file = "/home/nics/my_hub/freq/";
    txt_file += std::to_string(Module);
    txt_file += "/";
    txt_file += std::to_string(Priority);
    txt_file += "-";
    txt_file += std::to_string(mID.ID4);
    txt_file += ".txt";
    // init source info
    string str;
    ifstream ifs;
    int freq;
    ifs.open(txt_file, ios::in);
    if (!ifs.is_open())
    {
        std::cout << "open file error" << std::endl;
        return false;
    }
    // 一直读入

    while (getline(ifs, str))
    {
        freq = std::stoi(str);
    }
    ifs.close();
    if (freq > 0)
    {
        write_txt(txt_file, std::to_string(freq - 1));
        return true;
    }
    else if (freq == 0)
    {
        // release source
        init_source(mID, cpu, gpu);
        write_txt(txt_file, std::to_string(freq - 1));
        return false;
    }
    else
    {
        return false;
    }
}

judge_table read_from_string(string str, int ID1, int ID2, int ID3, int ID4)
{
    judge_table table;
    if (str.length() != 13)
    {
        cout << "error" << endl;
        table.ModuleID = -1;
        table.PriorityID = -1;
        table.Latency = -1;
        table.CPUCost = -1;
        table.GPUCost = -1;
    }
    else
    {
        table.ModuleID = std::stoi(str.substr(0, 2));
        table.PriorityID = std::stoi(str.substr(2, 2));
        table.Latency = (double)(std::stoi(str.substr(4, 3)) / 1000.0);
        table.CPUCost = (double)(std::stoi(str.substr(7, 3)) / 1000.0);
        table.GPUCost = (double)(std::stoi(str.substr(10, 3)) / 1000.0);
        table.MachineID.ID1 = ID1;
        table.MachineID.ID2 = ID2;
        table.MachineID.ID3 = ID3;
        table.MachineID.ID4 = ID4;
        // init_source(table.MachineID, table.ModuleID, table.PriorityID);
    }
    return table;
}

double ComputeHardwareFreq(judge_table table)
{
    bool run = table.Running;

    if (run)
    {
        return (1 / (double)table.Latency);
    }
    else
    {
        source_table source = read_source_from_txt(table.MachineID);
        if (table.CPUCost < source.CPUIdle && table.GPUCost < source.GPUIdle)
        {
            double freq = (1 / (double)table.Latency);

            return freq;
        }
        else
        {
            return 0;
        }
    }
}

int string_to_int(std::string s)
{
    // judege the string is int or not
    if (s.empty())
    {
        return 0;
    }
    if (s[0] == 'U')
    {
        return 0;
    }
    if (s[0] == 'e')
    {
        return 0;
    }
    int n = 0;

    for (int i = 0; i < s.length(); i++)
    {
        n = n * 10 + s[i] - '0';
    }
    return n;
}

long int string_to_long(std::string s)
{
    // judege the string is int or not
    if (s.empty())
    {
        return 0;
    }
    if (s[0] == 'U')
    {
        return 0;
    }
    if (s[0] == 'e')
    {
        return 0;
    }
    long int n = 0;

    for (int i = 0; i < s.length(); i++)
    {
        n = n * 10 + s[i] - '0';
    }
    return n;
}

namespace eprosima
{
    namespace fastrtps
    {
        namespace rtps
        {

            RTPSWriter::RTPSWriter(
                RTPSParticipantImpl *impl,
                const GUID_t &guid,
                const WriterAttributes &att,
                WriterHistory *hist,
                WriterListener *listen)
                : Endpoint(impl, guid, att.endpoint), mp_history(hist), mp_listener(listen), is_async_(att.mode == SYNCHRONOUS_WRITER ? false : true), locator_selector_(att.matched_readers_allocation), all_remote_readers_(att.matched_readers_allocation), all_remote_participants_(att.matched_readers_allocation), liveliness_kind_(att.liveliness_kind), liveliness_lease_duration_(att.liveliness_lease_duration), liveliness_announcement_period_(att.liveliness_announcement_period)
            {
                PoolConfig cfg = PoolConfig::from_history_attributes(hist->m_att);
                std::shared_ptr<IChangePool> change_pool;
                std::shared_ptr<IPayloadPool> payload_pool;
                payload_pool = BasicPayloadPool::get(cfg, change_pool);

                init(payload_pool, change_pool);
            }

            RTPSWriter::RTPSWriter(
                RTPSParticipantImpl *impl,
                const GUID_t &guid,
                const WriterAttributes &att,
                const std::shared_ptr<IPayloadPool> &payload_pool,
                WriterHistory *hist,
                WriterListener *listen)
                : RTPSWriter(
                      impl, guid, att, payload_pool,
                      std::make_shared<CacheChangePool>(PoolConfig::from_history_attributes(hist->m_att)),
                      hist, listen)
            {
            }

            RTPSWriter::RTPSWriter(
                RTPSParticipantImpl *impl,
                const GUID_t &guid,
                const WriterAttributes &att,
                const std::shared_ptr<IPayloadPool> &payload_pool,
                const std::shared_ptr<IChangePool> &change_pool,
                WriterHistory *hist,
                WriterListener *listen)
                : Endpoint(impl, guid, att.endpoint), mp_history(hist), mp_listener(listen), is_async_(att.mode == SYNCHRONOUS_WRITER ? false : true), locator_selector_(att.matched_readers_allocation), all_remote_readers_(att.matched_readers_allocation), all_remote_participants_(att.matched_readers_allocation), liveliness_kind_(att.liveliness_kind), liveliness_lease_duration_(att.liveliness_lease_duration), liveliness_announcement_period_(att.liveliness_announcement_period)
            {
                init(payload_pool, change_pool);
            }

            void RTPSWriter::init(
                const std::shared_ptr<IPayloadPool> &payload_pool,
                const std::shared_ptr<IChangePool> &change_pool)
            {
                payload_pool_ = payload_pool;
                change_pool_ = change_pool;
                fixed_payload_size_ = 0;
                if (mp_history->m_att.memoryPolicy == PREALLOCATED_MEMORY_MODE)
                {
                    fixed_payload_size_ = mp_history->m_att.payloadMaxSize;
                }

                mp_history->mp_writer = this;
                mp_history->mp_mutex = &mp_mutex;

                logInfo(RTPS_WRITER, "RTPSWriter created");
            }

            RTPSWriter::~RTPSWriter()
            {
                logInfo(RTPS_WRITER, "RTPSWriter destructor");

                // Deletion of the events has to be made in child destructor.

                for (auto it = mp_history->changesBegin(); it != mp_history->changesEnd(); ++it)
                {
                    release_change(*it);
                }

                mp_history->mp_writer = nullptr;
                mp_history->mp_mutex = nullptr;
            }

            CacheChange_t *RTPSWriter::new_change(
                const std::function<uint32_t()> &dataCdrSerializedSize,
                ChangeKind_t changeKind,
                InstanceHandle_t handle)
            {
                logInfo(RTPS_WRITER, "Creating new change");

                std::lock_guard<RecursiveTimedMutex> guard(mp_mutex);
                CacheChange_t *reserved_change = nullptr;
                if (!change_pool_->reserve_cache(reserved_change))
                {
                    logWarning(RTPS_WRITER, "Problem reserving cache from pool");
                    return nullptr;
                }

                uint32_t payload_size = fixed_payload_size_ ? fixed_payload_size_ : dataCdrSerializedSize();
                if (!payload_pool_->get_payload(payload_size, *reserved_change))
                {
                    change_pool_->release_cache(reserved_change);
                    logWarning(RTPS_WRITER, "Problem reserving payload from pool");
                    return nullptr;
                }

                reserved_change->kind = changeKind;
                if (m_att.topicKind == WITH_KEY && !handle.isDefined())
                {
                    logWarning(RTPS_WRITER, "Changes in KEYED Writers need a valid instanceHandle");
                }
                reserved_change->instanceHandle = handle;
                reserved_change->writerGUID = m_guid;
                return reserved_change;
            }

            bool RTPSWriter::release_change(
                CacheChange_t *change)
            {
                // Asserting preconditions
                assert(change != nullptr);
                assert(change->writerGUID == m_guid);

                std::lock_guard<RecursiveTimedMutex> guard(mp_mutex);

                IPayloadPool *pool = change->payload_owner();
                if (pool)
                {
                    pool->release_payload(*change);
                }
                return change_pool_->release_cache(change);
            }

            SequenceNumber_t RTPSWriter::get_seq_num_min()
            {
                CacheChange_t *change;
                if (mp_history->get_min_change(&change) && change != nullptr)
                {
                    return change->sequenceNumber;
                }
                else
                {
                    return c_SequenceNumber_Unknown;
                }
            }

            SequenceNumber_t RTPSWriter::get_seq_num_max()
            {
                CacheChange_t *change;
                if (mp_history->get_max_change(&change) && change != nullptr)
                {
                    return change->sequenceNumber;
                }
                else
                {
                    return c_SequenceNumber_Unknown;
                }
            }

            uint32_t RTPSWriter::getTypeMaxSerialized()
            {
                return mp_history->getTypeMaxSerialized();
            }

            bool RTPSWriter::remove_older_changes(
                unsigned int max)
            {
                logInfo(RTPS_WRITER, "Starting process clean_history for writer " << getGuid());
                std::lock_guard<RecursiveTimedMutex> guard(mp_mutex);
                bool limit = (max != 0);

                bool remove_ret = mp_history->remove_min_change();
                bool at_least_one = remove_ret;
                unsigned int count = 1;

                while (remove_ret && (!limit || count < max))
                {
                    remove_ret = mp_history->remove_min_change();
                    ++count;
                }

                return at_least_one;
            }

            CONSTEXPR uint32_t info_dst_message_length = 16;
            CONSTEXPR uint32_t info_ts_message_length = 12;
            CONSTEXPR uint32_t data_frag_submessage_header_length = 36;

            uint32_t RTPSWriter::getMaxDataSize()
            {
                return calculateMaxDataSize(mp_RTPSParticipant->getMaxMessageSize());
            }

            uint32_t RTPSWriter::calculateMaxDataSize(
                uint32_t length)
            {
                uint32_t maxDataSize = mp_RTPSParticipant->calculateMaxDataSize(length);

                maxDataSize -= info_dst_message_length +
                               info_ts_message_length +
                               data_frag_submessage_header_length;

                // TODO(Ricardo) inlineqos in future.

#if HAVE_SECURITY
                if (getAttributes().security_attributes().is_submessage_protected)
                {
                    maxDataSize -= mp_RTPSParticipant->security_manager().calculate_extra_size_for_rtps_submessage(m_guid);
                }

                if (getAttributes().security_attributes().is_payload_protected)
                {
                    maxDataSize -= mp_RTPSParticipant->security_manager().calculate_extra_size_for_encoded_payload(m_guid);
                }
#endif // if HAVE_SECURITY

                return maxDataSize;
            }

            void RTPSWriter::add_guid(
                const GUID_t &remote_guid)
            {
                const GuidPrefix_t &prefix = remote_guid.guidPrefix;
                all_remote_readers_.push_back(remote_guid);
                if (std::find(all_remote_participants_.begin(), all_remote_participants_.end(), prefix) ==
                    all_remote_participants_.end())
                {
                    all_remote_participants_.push_back(prefix);
                }
            }

            void RTPSWriter::compute_selected_guids()
            {
                all_remote_readers_.clear();
                all_remote_participants_.clear();

                for (LocatorSelectorEntry *entry : locator_selector_.transport_starts())
                {
                    GUID_t temp_id = entry->remote_guid;
                    std::string temp_topic_key = temp_id.instanceId;
                    int temp_topic_key_size = string_to_int(temp_topic_key);
                    // if (temp_topic_key_size > 20)
                    // {
                    //     if (temp_topic_key_size > 1000)
                    //     {
                    //         entry->enabled = false;
                    //         // entry->reset();
                    //         std::cout << "now_topic_key_int: " << temp_topic_key_size << std::endl;
                    //     }
                    // }
                    if (entry->enabled)
                    {
                        add_guid(entry->remote_guid);
                    }
                }
            }

            void RTPSWriter::update_cached_info_nts()
            {
                locator_selector_.reset(true);
                mp_RTPSParticipant->network_factory().select_locators(locator_selector_);
            }

            bool RTPSWriter::destinations_have_changed() const
            {
                return false;
            }

            GuidPrefix_t RTPSWriter::destination_guid_prefix() const
            {
                return all_remote_participants_.size() == 1 ? all_remote_participants_.at(0) : c_GuidPrefix_Unknown;
            }

            const std::vector<GuidPrefix_t> &RTPSWriter::remote_participants() const
            {
                return all_remote_participants_;
            }

            const std::vector<GUID_t> &RTPSWriter::remote_guids() const
            {
                return all_remote_readers_;
            }

            // judge whether the reader should be notified about the change
            bool Is_matched(
                const GUID_t &writer_guid,
                const GUID_t &reader_guid)
            {
                if (reader_guid.guidPrefix == c_GuidPrefix_Unknown)
                {
                    return true;
                }
                else if (reader_guid.guidPrefix == writer_guid.guidPrefix)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }

            bool RTPSWriter::send(
                CDRMessage_t *message,
                std::chrono::steady_clock::time_point &max_blocking_time_point) const
            {
                RTPSParticipantImpl *participant = getRTPSParticipant();
                long locator_size = sizeof(locator_selector_.entries_);
                LocatorSelector locator_selector_temp = locator_selector_;
                LocatorSelector locator_selector_temp2(1);
                int write_flag = 0;
                int write_int = 0;
                int calculate_flag = 0;
                int count = 0;
                vector<judge_table> tables[100];
                vector<machineID> mIDs[100][100];
                vector<judge_table> tablesrunning;
                int min = 999;
                int max = -99;
                int freq_demand = 5 * 46;
                if (locator_size > 0)
                {
                    for (LocatorSelectorEntry *entry : locator_selector_temp.entries_)
                    {
                        GUID_t temp_id = entry->remote_guid;
                        std::string temp_topic_key = temp_id.instanceId;
                        int temp_topic_key_size = temp_topic_key.length();
                        if (temp_topic_key_size == 13)
                        {
                            write_flag = 1;
                            string ModuleID = temp_topic_key.substr(0, 2);
                            int index = std::stoi(ModuleID);
                            string PriorityID = temp_topic_key.substr(2, 2);
                            int index2 = std::stoi(PriorityID);
                            if (min > index)
                            {
                                min = index;
                            }
                            if (max < index)
                            {
                                max = index;
                            }
                            tables[index].push_back(read_from_string(temp_topic_key, entry->unicast[0].address[12], entry->unicast[0].address[13], entry->unicast[0].address[14], entry->unicast[0].address[15]));
                            mIDs[index][index2].push_back(tables[index][tables[index].size() - 1].MachineID);
                        }
                    }
                }

                if (write_flag == 1)
                {
                    string count_str = read_txt("/home/nics/my_hub/count.txt");
                    count = std::stoi(count_str);
                    string calculate_flag_str = read_txt("/home/nics/my_hub/calculate.txt");
                    calculate_flag = std::stoi(calculate_flag_str);
                    struct timeval tv;
                    gettimeofday(&tv, NULL);
                    string time_tv = std::to_string(tv.tv_sec) + "." + std::to_string(tv.tv_usec);
                    string time_str = "/home/nics/my_hub/time/" + count_str + ".txt";
                    write_txt(time_str, time_tv);
                    if (calculate_flag == 0)
                    {
                        // init the sub's info
                        for (int i = min; i <= max; i++)
                        {
                            for (int j = 0; j < tables[i].size(); j++)
                            {
                                tables[i][j].Running = GetRunningInfo(tables[i][j].ModuleID, tables[i][j].PriorityID, tables[i][j].MachineID, tables[i][j].CPUCost, tables[i][j].GPUCost);
                                if (tables[i][j].Running == 1)
                                {
                                    tablesrunning.push_back(tables[i][j]);
                                    break;
                                }
                            }
                        }
                        count++;
                        if (count == freq_demand)
                        {
                            count = 0;
                            calculate_flag = 1;
                            write_txt("/home/nics/my_hub/calculate.txt", "1");
                        }
                        write_txt("/home/nics/my_hub/count.txt", std::to_string(count));
                    }

                    // calculate
                    else
                    {
                        // init the sub's info
                        for (int i = min; i <= max; i++)
                        {
                            for (int j = 0; j < tables[i].size(); j++)
                            {
                                tables[i][j].Running = GetRunningInfo(tables[i][j].ModuleID, tables[i][j].PriorityID, tables[i][j].MachineID, tables[i][j].CPUCost, tables[i][j].GPUCost);
                            }
                            // sort
                            sort(tables[i].begin(), tables[i].end(), compare);
                        }

                        for (int i = min; i <= max; i++)
                        {

                            if (tables[i].empty() != 1)
                            {
                                std::cout << "The table is not empty " << tables[i].empty() << endl;
                                // The length of the table
                                std::cout << "The length of the table is " << tables[i].size() << endl;
                                int execp = tables[i].size();
                                double temp_freq;
                                for (int j = 0; j < execp; j++)
                                {
                                    vector<double> freqs_double;
                                    vector<int> freqs;
                                    double max_freq = 0.0;
                                    int pID = tables[i][j].PriorityID;
                                    int pCount = mIDs[i][pID].size();
                                    int temp_j = j;
                                    for (int k = 0; k < pCount; k++)
                                    {
                                        // 对于一个算法而言，它可以有多个机器来运行。
                                        // 需要计算该算法下的所有机器的帧率，如果满足需求，则运行

                                        if (max_freq < freq_demand)
                                        {
                                            temp_freq = ComputeHardwareFreq(tables[i][j + k]) * 46;
                                            // 对于一个算法而言，它可以有多个机器来运行。
                                            // 需要计算该算法下的所有机器的帧率，如果满足需求，则运行
                                            if (tables[i][j + k].Running == false && temp_freq > 0.01)
                                            {
                                                // 需要消耗掉特定的cpu、GPU资源

                                                // 将该节点记为已经执行
                                                tables[i][j + k].Running = true;
                                            }

                                            else if (tables[i][j + k].Running == true)
                                            {
                                                // 认为这个节点需要加入到执行队列中
                                                // tablesrunning.push_back(tables[i][j + k]);
                                                cout << (1) << endl;
                                            }
                                            max_freq += temp_freq;
                                            freqs_double.push_back(temp_freq);
                                        }

                                        if (max_freq > freq_demand)
                                        {
                                            // 认为这个节点需要加入到执行队列中
                                            for (int l = 0; l <= k; l++)
                                            {
                                                cout << (1);
                                                modify_source(tables[i][j + l]);
                                                tablesrunning.push_back(tables[i][j + l]);
                                            }
                                            j = j + pCount - 1;
                                            break;
                                        }
                                        if (k == pCount - 1)
                                        {
                                            j = j + pCount - 1;
                                        }
                                    }
                                    if (max_freq > freq_demand)
                                    {
                                        for (int l = 0; l < freqs_double.size(); l++)
                                        {
                                            double sum = max_freq;
                                            int temp_freq = (int)(round(freqs_double[l] * freq_demand / sum));
                                            int temp_ModulID = tables[i][temp_j + l].ModuleID;
                                            int temp_PriorityID = tables[i][temp_j + l].PriorityID;
                                            machineID temp_machineID = tables[i][temp_j + l].MachineID;
                                            string freq_str = "/home/nics/my_hub/freq/" + to_string(temp_ModulID) + "/" + to_string(temp_PriorityID) + "-" + to_string(temp_machineID.ID4) + ".txt";
                                            write_txt(freq_str, to_string(temp_freq));
                                            freqs.push_back((int)(round(freqs_double[l] * freq_demand / sum)));
                                        }
                                        break;
                                    }
                                }
                            }
                        }

                        calculate_flag = 0;
                        write_txt("/home/nics/my_hub/calculate.txt", "0");
                    }
                }

                if (write_flag == 1)
                {
                    for (LocatorSelectorEntry *entry : locator_selector_temp.entries_)
                    {
                        GUID_t temp_id = entry->remote_guid;
                        std::string temp_topic_key = temp_id.instanceId;
                        int ModuleID = std::stoi(temp_topic_key.substr(0, 2));
                        int PriorityID = std::stoi(temp_topic_key.substr(2, 2));
                        // auto ID4 = entry->unicast[0]->address[15];
                        auto IP = entry->unicast;
                        int ID4 = IP[0].address[15];
                        for (int i = 0; i < tablesrunning.size(); i++)
                        {
                            if (ModuleID == tablesrunning[i].ModuleID && PriorityID == tablesrunning[i].PriorityID && ID4 == tablesrunning[i].MachineID.ID4)
                            {
                                locator_selector_temp2.add_entry_tmp(entry, 0);
                            }
                        }
                    }
                }

                // locator_selector_temp2.add_entry_tmp(entry, 0);
                locator_selector_temp2.fresh_selections();

                // 此处有golbal信息，可以在此添加筛选条件
                if (write_flag == 1)
                {
                    return locator_selector_temp2.selected_size() == 0 ||
                           // participant->sendSync(message, locator_selector_.begin(), locator_selector_.begin(), max_blocking_time_point);
                           participant->sendSync(message, locator_selector_temp2.begin(), locator_selector_temp2.end(), max_blocking_time_point);
                }
                else
                {
                    return locator_selector_temp.selected_size() == 0 ||
                           // participant->sendSync(message, locator_selector_.begin(), locator_selector_.begin(), max_blocking_time_point);
                           participant->sendSync(message, locator_selector_temp.begin(), locator_selector_temp.end(), max_blocking_time_point);
                }
            }

            const LivelinessQosPolicyKind &RTPSWriter::get_liveliness_kind() const
            {
                return liveliness_kind_;
            }

            const Duration_t &RTPSWriter::get_liveliness_lease_duration() const
            {
                return liveliness_lease_duration_;
            }

            const Duration_t &RTPSWriter::get_liveliness_announcement_period() const
            {
                return liveliness_announcement_period_;
            }

        } // namespace rtps
    }     // namespace fastrtps
} // namespace eprosima
